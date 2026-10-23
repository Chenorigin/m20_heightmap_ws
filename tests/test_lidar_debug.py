"""Viewer contract regressions: malformed/stale data must never become valid floor."""
import argparse
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest


def load_script(name):
    path = Path(__file__).resolve().parents[1] / "scripts" / (name + ".py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


collector = load_script("stream_lidar_debug")
viewer = load_script("view_lidar_debug")


def message():
    # schema=2 and age=-1 for unknown are actual production wire semantics.
    return NS(schema_version=2, profile="v1", header=NS(frame_id="base_yaw", stamp=NS(sec=100, nanosec=0)),
              sequence=7, session_id=2**63+1, geometry_hash="test", motion_compensated=True,
              operator_commissioned=False, height=[0.] * 187, validity=[0.] * 187,
              confidence=[0.] * 187, variance=[0.] * 187, age=[-1.] * 187)


def test_schema_and_unknown_sentinel():
    msg = message()
    cfg = {"profile": "v1", "frames": {"output": "base_yaw"}}
    out = collector.terrain_payload(msg, cfg)
    assert out["age"] == [-1.] * 187
    assert out["session"] == str(2**63+1)
    msg.schema_version = 1
    with pytest.raises(ValueError, match="schema"):
        collector.terrain_payload(msg, cfg)


@pytest.mark.parametrize("name,value", [("height", float("nan")), ("validity", .5),
                                        ("confidence", 1.1), ("variance", -.1)])
def test_malformed_terrain_rejected(name, value):
    msg = message()
    getattr(msg, name)[0] = value
    with pytest.raises(ValueError):
        collector.terrain_payload(msg, {"profile": "v1", "frames": {"output": "base_yaw"}})


def test_cell_and_frame_aging_and_future():
    t = {k: [0.] * 187 for k in ("height", "validity", "confidence", "variance", "age")}
    t["validity"][0:2], t["age"][0:2] = [1., 1.], [.2, .49]
    t["header_age_s"] = .02
    limits = {"max_cell_age_s": .5, "terrain_frame_age_s": .12, "future_tolerance_s": .005}
    assert viewer.cell_state(t, 0, limits)[1].sum() == 1
    assert not viewer.cell_state(t, .2, limits)[1].any()
    t["header_age_s"] = -.01
    assert viewer.cell_state(t, 0, limits)[4]
    assert not viewer.cell_state(t, 0, limits)[1].any()


def test_cloud_downsampling_preserves_native_frame_and_finite_points():
    msg = NS(width=200, height=1, header=NS(frame_id="lidar_link", stamp=NS(sec=9, nanosec=10)))
    xyz = np.arange(600, dtype=float).reshape(-1, 3)
    xyz[0, 0] = np.nan
    out = collector.sample_cloud(msg, lambda *a, **k: (xyz, None), 100,
                                 {"runtime": {"max_cloud_points": 200000, "max_cloud_bytes": 16000000}})
    assert out["frame"] == "lidar_link"
    assert out["source_points"] == 200 and out["finite_points"] == 199
    assert out["shown_points"] == 100 and np.isfinite(out["xyz"]).all()
    assert out["xyz"][0] == xyz[1].tolist() and out["xyz"][-1] == xyz[-1].tolist()


def test_ssh_does_not_interpolate_paths_or_disable_host_verification():
    args = argparse.Namespace(config="runtime/a'b.json", seconds=10, hz=2, max_points=3000,
                              release="/release/$(never_run)", jump="gateway", ssh_target="gos")
    argv = viewer.ssh_command(args)
    assert "StrictHostKeyChecking=yes" in argv and "BatchMode=yes" in argv
    assert argv[-2] == "gos" and "'" in argv[-1]
    assert "sudo" not in argv[-1] and "operator_start" not in argv[-1]


def test_transport_end_expires_last_live_frame():
    stream = viewer.Stream()
    import io
    stream.read(io.StringIO('{"schema":"m20_lidar_debug_v1"}\n'))
    assert stream.snapshot()[2].startswith("连接已结束")
    stream.read(io.StringIO('{"schema":"bad"}\n'))
    assert stream.snapshot()[2].startswith("数据错误")


def test_explicit_ssh_user_wins_over_alias_default():
    assert viewer.ssh_address("specific@127.0.0.1") == ("127.0.0.1", "specific", 22)


def test_front_origin_comes_from_unique_forward_installation():
    cfg = {"visibility": {"sensors": [
        {"position_base_m": [-.32, 0., -.013], "yaw_rad": np.pi},
        {"position_base_m": [.32, 0., -.013], "yaw_rad": 0.}]}}
    np.testing.assert_allclose(collector.front_position(cfg), [.32, 0., -.013])
    cfg["visibility"]["sensors"].append(cfg["visibility"]["sensors"][-1])
    with pytest.raises(ValueError, match="exactly one"):
        collector.front_position(cfg)


def test_cloud_origin_roundtrip_and_pairwise_distance_invariant():
    from m20_heightmap.geometry import rotation
    cfg = {"lidar_extrinsics": {"native": {
        "translation_m": [.1, -.2, .3], "quaternion_xyzw": [0., 0., np.sin(.3), np.cos(.3)]}}}
    front = np.array([.32, 0., -.013])
    origins = collector.cloud_origins("native", cfg, front, rotation)
    ext = cfg["lidar_extrinsics"]["native"]
    R = rotation(ext["quaternion_xyzw"])
    np.testing.assert_allclose(R @ origins["front_lidar_origin_m"] + ext["translation_m"], front)
    points = np.random.default_rng(10).normal(size=(100, 3))
    before = points.copy()
    front_points = points - viewer.origin_offset(origins, "front")
    np.testing.assert_allclose(front_points[1:] - front_points[:-1], points[1:] - points[:-1])
    np.testing.assert_allclose(front_points + viewer.origin_offset(origins, "front"), before, atol=1e-15)
    np.testing.assert_array_equal(points, before)


def test_terrain_front_lever_includes_roll_pitch_and_is_yaw_invariant():
    from m20_heightmap.geometry import Pose, yaw_rotation
    front = np.array([.32, 0., -.013])
    pitch = .4
    R = np.array([[np.cos(pitch), 0., np.sin(pitch)], [0., 1., 0.], [-np.sin(pitch), 0., np.cos(pitch)]])
    expected = R @ front
    for yaw in (0., .3, -2.):
        pose = NS(R=yaw_rotation(yaw) @ R, yaw=yaw)
        np.testing.assert_allclose(collector.terrain_origin(front, pose, yaw_rotation), expected, atol=1e-15)
    assert abs(expected[2] - front[2]) > .1
    level = Pose(1., np.zeros(3), [0., 0., 0., 1.], np.zeros(6))
    np.testing.assert_allclose(collector.terrain_origin(front, level, yaw_rotation), front)


@pytest.mark.parametrize("offset", [[0., 0., 0.], [.32, -.015, -.013], [.2, .1, -.15]])
def test_grid_origin_keeps_indices_and_hover_mapping(offset):
    offset = np.array(offset)
    center, scale = viewer.grid_layout(640, 440, offset)
    for iy in range(11):
        for ix in range(17):
            pixel = viewer.grid_pixel([-.8 + ix * .1, -.5 + iy * .1], center, scale, offset)
            assert viewer.grid_cell(pixel, center, scale, offset) == (ix, iy)
    np.testing.assert_allclose(viewer.grid_pixel(offset[:2], center, scale, offset), center)
    assert viewer.grid_cell([-100., -100.], center, scale, offset) is None


def test_front_origin_missing_metadata_fails_explicitly():
    with pytest.raises(ValueError, match="缺少"):
        viewer.origin_offset({}, "front")
    np.testing.assert_array_equal(viewer.origin_offset({}, "body"), np.zeros(3))


def test_shadow_source_selection_never_falls_back_or_mutates():
    baseline, candidate, published = {"height": [1.]}, {"height": [2.]}, {"height": [3.]}
    packet = {"streams": {"terrain": baseline, "terrain_baseline": baseline,
                          "terrain_candidate": candidate, "terrain_published": published}}
    for name, expected in [("default",baseline),("baseline",baseline),("candidate",candidate),("published",published)]:
        assert viewer.selected_terrain(packet,name) is expected
    assert packet["streams"]["terrain"]["height"] == [1.]
    assert viewer.selected_terrain({"streams":{"terrain":baseline}},"candidate") is None
    assert viewer.selected_terrain(None,"published") is None
    with pytest.raises(ValueError):
        viewer.selected_terrain(packet,"other")


def test_display_and_hover_share_disconnect_and_replay_aging():
    t={k:[0.]*187 for k in ('height','validity','confidence','variance','age')}
    t['validity'][0]=1.;t['age'][0]=.1;t['header_age_s']=.02
    limits={'max_cell_age_s':.5,'terrain_frame_age_s':.12,'future_tolerance_s':.005}
    assert viewer.displayed_cell_state(t,0.,limits,2.,False)[1][0]
    assert viewer.displayed_cell_state(t,0.,limits,2.,True)[1][0]
    stale=viewer.displayed_cell_state(t,2.,limits,2.,False)
    assert stale[-1] and not stale[1].any()
    t['silence_s']=2.
    assert not viewer.displayed_cell_state(t,0.,limits,2.,False)[1].any()


def test_same_frame_height_difference_requires_common_fresh_support():
    import copy
    base = collector.terrain_payload(message(), {"profile":"v1","frames":{"output":"base_yaw"}})
    base.update(header_age_s=0., silence_s=0.)
    base['validity'][0]=1.;base['age'][0]=.1
    candidate=copy.deepcopy(base);candidate['height'][0]=.005
    candidate['validity'][1]=1.;candidate['age'][1]=.1
    packet={'streams':{'terrain_baseline':base,'terrain_candidate':candidate},
            'limits':{'max_cell_age_s':.5,'terrain_frame_age_s':.12,'future_tolerance_s':.005}}
    original=copy.deepcopy(packet)
    delta,known=viewer.height_difference(packet,'candidate',0.,2.,False)
    assert known.sum()==1 and delta[0]==pytest.approx(5.)
    assert not viewer.height_difference(packet,'candidate',.2,2.,True)[1].any()
    assert packet==original
    candidate['sequence']+=1
    with pytest.raises(ValueError,match='不一致'):
        viewer.height_difference(packet,'candidate',0.,2.,False)
    del packet['streams']['terrain_baseline']
    with pytest.raises(ValueError,match='缺少'):
        viewer.height_difference(packet,'candidate',0.,2.,False)
