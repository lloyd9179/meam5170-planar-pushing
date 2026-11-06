import numpy as np
import pytest

from planar_pushing import PlanarPushingEnv


def active_coefficients(env):
    groups = {"ground_slider": [], "pusher_slider": []}
    for contact in env.contact_info():
        key = "ground_slider" if "table" in contact["geoms"] else "pusher_slider"
        assert contact["dim"] == 3
        assert contact["normal_force"] > 0
        groups[key].append(contact["friction"])
    assert all(groups.values()), "both pairs must actually be in loaded contact"
    return groups


def check_actual_contacts(env, ground, pusher):
    for key, mu in (("ground_slider", ground), ("pusher_slider", pusher)):
        pair = env.model.pair(key).id
        np.testing.assert_allclose(env.model.pair_friction[pair, :2], mu)
    actual = active_coefficients(env)
    for key, mu in (("ground_slider", ground), ("pusher_slider", pusher)):
        for vector in actual[key]:
            np.testing.assert_allclose(vector, [mu, mu, .001, .0001, .0001], rtol=0, atol=1e-12)


def test_independent_friction_on_real_contacts():
    env = PlanarPushingEnv()
    for _ in range(35):
        env.step([.025, 0])
    check_actual_contacts(env, .3, .5)
    # Deliberately set huge individual geom friction: explicit pairs must prevail.
    env.model.geom_friction[:] = [9, .9, .09]
    env.set_friction(mu_ground=.12)
    env.step([.025, 0])
    check_actual_contacts(env, .12, .5)
    env.set_friction(mu_pusher=.83)
    env.step([.025, 0])
    check_actual_contacts(env, .12, .83)
    env.set_friction(mu_ground=.62)
    env.step([.025, 0])
    check_actual_contacts(env, .62, .83)
    assert env.get_friction()["mu_ground"] == .62
    assert env.get_friction()["mu_pusher"] == .83
    env.reset()  # configured true friction persists across episodes
    assert env.get_friction()["mu_ground"] == .62


@pytest.mark.parametrize("bad", [-.1, np.nan, np.inf, [1, 2]])
def test_invalid_friction_is_atomic(bad):
    env = PlanarPushingEnv()
    before = env.get_friction()
    with pytest.raises(ValueError):
        env.set_friction(mu_ground=.9, mu_pusher=bad)
    assert env.get_friction() == before
