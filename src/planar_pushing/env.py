"""MuJoCo execution dynamics, independent of any controller or prediction model."""

from dataclasses import dataclass
from importlib.resources import files
from math import atan2, cos, sin
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def _vector(value, length, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must have shape ({length},) and finite values")
    return result


@dataclass(frozen=True)
class EnvConfig:
    """Dimensions are full lengths (m), poses are world (x, y, yaw in rad)."""

    slider_size: tuple[float, float, float] = (0.12, 0.08, 0.02)
    slider_mass: float = 0.2
    slider_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
    pusher_position: tuple[float, float] | None = None
    pusher_radius: float = 0.01
    pusher_height: float = 0.016
    goal_pose: tuple[float, float, float] = (0.12, 0.03, 0.4)
    mu_ground: float = 0.3
    mu_pusher: float = 0.5
    dt_physics: float = 0.001
    dt_control: float = 0.02
    max_pusher_speed: float = 0.08
    pusher_mode: str = "mocap"
    pusher_servo_gain: float = 1000.0  # N/(m/s); optional real XY velocity servo
    pusher_mass: float = 0.1

    def __post_init__(self):
        sizes = _vector(self.slider_size, 3, "slider_size")
        if np.any(sizes <= 0):
            raise ValueError("slider_size must be positive")
        for name in ("slider_mass", "pusher_radius", "pusher_height", "dt_physics",
                     "dt_control", "max_pusher_speed", "pusher_servo_gain", "pusher_mass"):
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("mu_ground", "mu_pusher"):
            value = getattr(self, name)
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        _vector(self.slider_pose, 3, "slider_pose")
        _vector(self.goal_pose, 3, "goal_pose")
        if self.pusher_position is not None:
            _vector(self.pusher_position, 2, "pusher_position")
        if self.pusher_height >= sizes[2]:
            raise ValueError("pusher_height must be less than slider height to clear table")
        ratio = self.dt_control / self.dt_physics
        if ratio < 1 or not np.isclose(ratio, round(ratio), rtol=0, atol=1e-10):
            raise ValueError("dt_control must be a positive integer multiple of dt_physics")
        if self.dt_physics > 0.002:
            raise ValueError("dt_physics must be <= 0.002 s for this contact configuration")
        if self.pusher_mode not in ("mocap", "velocity_servo"):
            raise ValueError("pusher_mode must be mocap or velocity_servo")


@dataclass(frozen=True)
class State:
    """World-frame snapshot: positions m, velocities m/s and rad/s, time s."""

    slider_position: tuple[float, float, float]
    theta: float  # yaw in [-pi, pi]; unwrap for long trajectories
    pusher_position: tuple[float, float]
    slider_linear_velocity: tuple[float, float, float]
    slider_angular_velocity: tuple[float, float, float]
    time: float

    @property
    def slider_pose(self):
        return (*self.slider_position[:2], self.theta)


class PlanarPushingEnv:
    """Free-body slider with table support and a velocity-commanded cylinder.

    reset(seed) -> State; step([vx, vy]) -> (State, info).
    No GUI or OpenGL context is created by this class until launch_viewer().
    An Env instance is not thread-safe; use one per simulation worker.
    """

    def __init__(self, config: EnvConfig | None = None):
        self.config = config or EnvConfig()
        c = self.config
        root = ET.fromstring(files("planar_pushing").joinpath("models/planar_pushing.xml").read_text())
        root.find("option").set("timestep", str(c.dt_physics))
        slider = root.find(".//body[@name='slider']")
        geom = slider.find("geom")
        geom.set("size", " ".join(str(v / 2) for v in c.slider_size))
        geom.set("mass", str(c.slider_mass))
        slider.find("site").set("pos", f"{c.slider_size[0] * .375} 0 {c.slider_size[2]/2 + .0005}")
        pusher = root.find(".//geom[@name='pusher_geom']")
        pusher.set("size", f"{c.pusher_radius} {c.pusher_height / 2}")
        if c.pusher_mode == "velocity_servo":
            body = root.find(".//body[@name='pusher']")
            body.attrib.pop("mocap")
            pusher.set("mass", str(c.pusher_mass))
            actuators = ET.SubElement(root, "actuator")
            for name, axis in (("x", "1 0 0"), ("y", "0 1 0")):
                ET.SubElement(body, "joint", name=f"pusher_{name}", type="slide", axis=axis)
                ET.SubElement(actuators, "velocity", name=f"pusher_v{name}",
                              joint=f"pusher_{name}", kv=str(c.pusher_servo_gain))
        goal = root.find(".//body[@name='goal']")
        goal.set("pos", f"{c.goal_pose[0]} {c.goal_pose[1]} 0.001")
        goal.set("euler", f"0 0 {c.goal_pose[2]}")
        hx, hy = np.asarray(c.slider_size[:2]) / 2
        corners = {"left": (-hx, -hy, -hx, hy), "right": (hx, -hy, hx, hy),
                   "bottom": (-hx, -hy, hx, -hy), "top": (-hx, hy, hx, hy),
                   "heading": (0, 0, hx * 2/3, 0)}
        for name, (x1, y1, x2, y2) in corners.items():
            goal.find(f"geom[@name='goal_{name}']").set("fromto", f"{x1} {y1} 0 {x2} {y2} 0")
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        self.data = mujoco.MjData(self.model)
        self._slider_id = self.model.body("slider").id
        self._mocap_id = int(self.model.body_mocapid[self.model.body("pusher").id])
        joint = self.model.joint("slider_free").id
        self._qadr = int(self.model.jnt_qposadr[joint])
        self.n_substeps = round(c.dt_control / c.dt_physics)
        self.set_friction(mu_ground=c.mu_ground, mu_pusher=c.mu_pusher)
        self.reset()

    def reset(self, seed=0, *, slider_pose=None, pusher_position=None):
        """Reset time, velocities, contacts, warm starts and mocap state.

        No randomization is enabled in Step 0. seed initializes a local RNG for
        reproducible future extensions, without changing NumPy's global RNG.
        Pusher defaults to 3 mm outside the slider's local -X face center.
        """
        pose = _vector(self.config.slider_pose if slider_pose is None else slider_pose, 3, "slider_pose")
        supplied = self.config.pusher_position if pusher_position is None else pusher_position
        if supplied is None:
            distance = self.config.slider_size[0] / 2 + self.config.pusher_radius + .003
            pxy = pose[:2] - distance * np.array([cos(pose[2]), sin(pose[2])])
        else:
            pxy = _vector(supplied, 2, "pusher_position")
        # Reject an overlapping initialization in the rectangle's local XY frame.
        delta = pxy - pose[:2]
        rot = np.array([[cos(pose[2]), sin(pose[2])], [-sin(pose[2]), cos(pose[2])]])
        outside = np.maximum(np.abs(rot @ delta) - np.asarray(self.config.slider_size[:2]) / 2, 0)
        if np.linalg.norm(outside) < self.config.pusher_radius - 1e-10:
            raise ValueError("initial pusher overlaps slider; choose a nonpenetrating pose")
        rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        self.rng = rng
        q = self._qadr
        # The only slider qpos writes in the environment are initial conditions.
        self.data.qpos[q:q+3] = (*pose[:2], self.config.slider_size[2] / 2 + .0002)
        self.data.qpos[q+3:q+7] = (cos(pose[2]/2), 0, 0, sin(pose[2]/2))
        if self.config.pusher_mode == "mocap":
            self.data.mocap_pos[self._mocap_id] = (*pxy, self.config.slider_size[2] / 2)
            self.data.mocap_quat[self._mocap_id] = (1, 0, 0, 0)
        else:
            for i, name in enumerate(("x", "y")):
                adr = self.model.jnt_qposadr[self.model.joint(f"pusher_{name}").id]
                self.data.qpos[adr] = pxy[i] - self.model.body("pusher").pos[i]
            self.model.body("pusher").pos[2] = self.config.slider_size[2] / 2
        mujoco.mj_forward(self.model, self.data)
        return self.get_state()

    def get_state(self):
        """Return a detached immutable state; callers need no qpos indexing."""
        w, x, y, z = self.data.xquat[self._slider_id]
        theta = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                self._slider_id, velocity, 0)
        return State(tuple(self.data.xpos[self._slider_id]), theta,
                     tuple(self.data.xpos[self.model.body("pusher").id, :2]),
                     tuple(velocity[3:]), tuple(velocity[:3]), float(self.data.time))

    def set_friction(self, mu_ground=None, mu_pusher=None):
        """Update explicit pairs atomically, leaving all unspecified values intact.

        condim=3 activates two tangential friction directions. Torsion/rolling
        coefficients are stored but inactive. Zero is clamped by MuJoCo to its
        small internal friction floor when constructing actual contacts.
        """
        updates = [("ground_slider", mu_ground), ("pusher_slider", mu_pusher)]
        for name, mu in updates:
            if mu is not None and (not np.isscalar(mu) or not np.isfinite(mu) or mu < 0):
                raise ValueError(f"{name} friction must be finite and nonnegative")
        for name, mu in updates:
            if mu is not None:
                self.model.pair_friction[self.model.pair(name).id, :2] = mu
        if hasattr(self, "data"):
            mujoco.mj_forward(self.model, self.data)
        return self.get_friction()

    def get_friction(self):
        """Read compiled model pairs, rather than cached Python configuration."""
        result = {}
        for key, name in (("mu_ground", "ground_slider"), ("mu_pusher", "pusher_slider")):
            idx = self.model.pair(name).id
            result[key] = float(self.model.pair_friction[idx, 0])
            result[name] = {"friction": self.model.pair_friction[idx].tolist(),
                            "condim": int(self.model.pair_dim[idx])}
        return result

    def contact_info(self):
        """Current actual contacts: geom names, friction, distance and normal force."""
        contacts = []
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force)
            contacts.append({"geoms": (self.model.geom(contact.geom1).name,
                                        self.model.geom(contact.geom2).name),
                             "friction": contact.friction.copy().tolist(),
                             "dim": int(contact.dim), "distance": float(contact.dist),
                             "normal_force": float(force[0])})
        return contacts

    def step(self, u):
        """Hold a world XY velocity for dt_control, with norm saturation.

        Mocap advances by u*dt_physics; optional velocity_servo sends the same
        saturated reference to physical XY joint velocity actuators.
        Only contact forces move the slider. info reports reference velocity,
        saturation, current contacts and worst contact distance over substeps.
        """
        requested = _vector(u, 2, "u")
        # Scale before taking a norm, so even finite inputs near 1e308 are safe.
        scale = float(np.max(np.abs(requested)))
        unit = requested / scale if scale else requested.copy()
        unit_norm = float(np.linalg.norm(unit))
        clipped = bool(scale > self.config.max_pusher_speed / unit_norm) if scale else False
        applied = unit * (self.config.max_pusher_speed / unit_norm) if clipped else requested.copy()
        minimum_distance = 0.0
        if self.config.pusher_mode == "velocity_servo":
            self.data.ctrl[:] = applied
        for _ in range(self.n_substeps):
            if self.config.pusher_mode == "mocap":
                self.data.mocap_pos[self._mocap_id, :2] += applied * self.config.dt_physics
            mujoco.mj_step(self.model, self.data)
            if self.data.ncon:
                minimum_distance = min(minimum_distance, float(np.min(self.data.contact.dist)))
        mujoco.mj_forward(self.model, self.data)
        if (not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel))
                or any(w.number for w in self.data.warning)):
            raise RuntimeError("MuJoCo reported an unstable simulation; reset and inspect parameters")
        return self.get_state(), {"requested_velocity": requested.tolist(),
                                  "applied_velocity": applied.tolist(), "speed_clipped": clipped,
                                  "contacts": self.contact_info(),
                                  "min_contact_distance": minimum_distance}

    def launch_viewer(self):
        """Return a passive viewer context; call sync after stepping (main thread)."""
        import mujoco.viewer
        return mujoco.viewer.launch_passive(self.model, self.data)
