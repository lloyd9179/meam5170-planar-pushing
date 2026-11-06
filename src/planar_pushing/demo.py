"""Prescribed Step 0 experiments; these are not MPC controllers."""

import argparse
from contextlib import nullcontext
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np

from .env import EnvConfig, PlanarPushingEnv


def run_demo(mode="center", *, duration=4.0, speed=.025, viewer=False, config=None, seed=0):
    """Run a constant +X push on the local -X side of the default slider pose."""
    if mode not in ("center", "offcenter"):
        raise ValueError("mode must be center or offcenter")
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be finite and positive")
    env = PlanarPushingEnv(config)
    if not np.isfinite(speed) or not 0 < speed <= env.config.max_pusher_speed:
        raise ValueError("speed must be positive and within max_pusher_speed")
    steps = round(duration / env.config.dt_control)
    if steps < 1 or not np.isclose(steps * env.config.dt_control, duration):
        raise ValueError("duration must be an integer multiple of dt_control")
    pose = np.asarray(env.config.slider_pose)
    yaw = pose[2]
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    local_pusher = np.array([-env.config.slider_size[0]/2 - env.config.pusher_radius - .003,
                             0 if mode == "center" else env.config.slider_size[1] * .3125])
    state = env.reset(seed, pusher_position=pose[:2] + rotation @ local_pusher)
    states = [state]
    worst_distance, contact_steps, peak_force = 0.0, 0, 0.0
    max_vertical_drift, max_tilt, viewer_sync_frames = 0.0, 0.0, 0
    with env.launch_viewer() if viewer else nullcontext() as window:
        if window is not None:
            window.cam.lookat[:] = (pose[0] + .05, pose[1], .01)
            window.cam.distance = .5
            window.cam.azimuth, window.cam.elevation = 110, -65
        for _ in range(steps):
            start = time.perf_counter()
            if window is not None and not window.is_running():
                break
            state, info = env.step(rotation @ np.array([speed, 0.0]))
            states.append(state)
            worst_distance = min(worst_distance, info["min_contact_distance"])
            pusher_contacts = [c for c in info["contacts"] if "pusher_geom" in c["geoms"]]
            contact_steps += bool(pusher_contacts)
            peak_force = max([peak_force] + [c["normal_force"] for c in pusher_contacts])
            max_vertical_drift = max(max_vertical_drift, abs(state.slider_position[2] - env.config.slider_size[2]/2))
            w, x, y, z = env.data.xquat[env._slider_id]
            max_tilt = max(max_tilt, float(np.arccos(np.clip(1 - 2*(x*x + y*y), -1, 1))))
            if window is not None:
                window.sync()
                viewer_sync_frames += 1
                time.sleep(max(0, env.config.dt_control - (time.perf_counter() - start)))
    initial, final = states[0], states[-1]
    delta_xy = np.asarray(final.slider_position[:2]) - initial.slider_position[:2]
    yaw_change = np.unwrap([s.theta for s in states])[-1] - initial.theta
    summary = {"mode": mode, "duration_s": final.time, "completed": len(states) == steps + 1,
               "viewer_requested": viewer, "viewer_sync_frames": viewer_sync_frames,
               "translation_m": float(np.linalg.norm(delta_xy)),
               "delta_x_m": float(delta_xy[0]), "delta_y_m": float(delta_xy[1]),
               "yaw_change_rad": float(yaw_change), "yaw_change_deg": float(np.degrees(yaw_change)),
               "pusher_contact_steps": contact_steps, "peak_pusher_normal_force_N": peak_force,
               "minimum_contact_distance_m": worst_distance,
               "max_vertical_drift_m": max_vertical_drift, "max_tilt_rad": max_tilt,
               "friction": env.get_friction(), "initial": asdict(initial), "final": asdict(final)}
    return states, summary


def save_trajectory(states, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    array = np.array([[s.time, *s.slider_position, s.theta, *s.pusher_position,
                       *s.slider_linear_velocity, *s.slider_angular_velocity] for s in states])
    np.savetxt(path, array, delimiter=",", comments="",
               header="time_s,slider_x_m,slider_y_m,slider_z_m,yaw_rad,pusher_x_m,pusher_y_m,slider_vx_m_s,slider_vy_m_s,slider_vz_m_s,slider_wx_rad_s,slider_wy_rad_s,slider_wz_rad_s")


def plot_comparison(trajectories, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True, constrained_layout=True)
    for mode, states in trajectories.items():
        t = [s.time for s in states]
        values = ([s.slider_position[0] for s in states],
                  [s.slider_position[1] for s in states], np.unwrap([s.theta for s in states]))
        for ax, data in zip(axes, values):
            ax.plot(t, data, label=mode, linewidth=2)
    for ax, ylabel in zip(axes, ("Slider x (m)", "Slider y (m)", "Slider yaw (rad)")):
        ax.set_ylabel(ylabel)
        ax.grid(alpha=.25)
        ax.legend()
    axes[-1].set_xlabel("Simulation time (s)")
    fig.suptitle("MuJoCo contact dynamics: center vs. off-center push")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("center", "offcenter", "both"), default="both")
    display = parser.add_mutually_exclusive_group()
    display.add_argument("--viewer", action="store_true", help="open passive MuJoCo viewer")
    display.add_argument("--headless", action="store_true", help="explicit headless mode (default)")
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--speed", type=float, default=.025)
    parser.add_argument("--mu-ground", type=float, default=.3)
    parser.add_argument("--mu-pusher", type=float, default=.5)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts"))
    args = parser.parse_args()
    trajectories, summaries = {}, {}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for mode in (("center", "offcenter") if args.mode == "both" else (args.mode,)):
        states, summary = run_demo(mode, duration=args.duration, speed=args.speed,
                                   viewer=args.viewer, seed=args.seed,
                                   config=EnvConfig(mu_ground=args.mu_ground, mu_pusher=args.mu_pusher))
        trajectories[mode], summaries[mode] = states, summary
        save_trajectory(states, args.output_dir / f"{mode}.csv")
    (args.output_dir / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n")
    plot_comparison(trajectories, args.output_dir / "trajectories.png")
    print(json.dumps(summaries, indent=2))


if __name__ == "__main__":
    main()
