#!/usr/bin/env python3
"""Bounded GOS subscribers only. JSON lines on stdout, diagnostics on stderr.

Can run as Python stdin over SSH: no installation and no board files written.
Only existing topics are observed; this never starts perception or the SDK.
"""
import argparse
import json
import os
from pathlib import Path
import time

import numpy as np


SCHEMA = "m20_lidar_debug_v1"


def front_position(cfg):
    sensors = cfg.get("visibility", {}).get("sensors", [])
    front = [s for s in sensors if s["position_base_m"][0] > 0 and np.cos(s["yaw_rad"]) > .5]
    if len(front) != 1:
        raise ValueError("front lidar installation must identify exactly one forward sensor")
    value = np.asarray(front[0]["position_base_m"], dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError("invalid front lidar installation")
    return value


def cloud_origins(frame, cfg, position, rotation):
    ext = cfg["lidar_extrinsics"][frame]
    inv = rotation(ext["quaternion_xyzw"]).T
    translation = np.asarray(ext["translation_m"])
    return {"body_origin_m": (inv @ -translation).tolist(),
            "front_lidar_origin_m": (inv @ (position - translation)).tolist()}


def terrain_origin(position, pose, yaw_rotation):
    # Keep gravity-aligned axes. Translate the measured front sensor lever arm
    # using the pose at this terrain query, including pitch/roll, never just yaw.
    return (yaw_rotation(pose.yaw).T @ pose.R @ position).tolist()


def sample_cloud(msg, read_cloud, max_points, cfg):
    points, _ = read_cloud(msg, max_points=cfg["runtime"]["max_cloud_points"],
                           max_bytes=cfg["runtime"]["max_cloud_bytes"])
    points = points[np.isfinite(points).all(axis=1)]
    count = len(points)
    if count > max_points:
        points = points[np.linspace(0, count - 1, max_points, dtype=np.int64)]
    return {"frame": msg.header.frame_id,
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "source_points": int(msg.width * msg.height), "finite_points": count,
            "shown_points": len(points), "xyz": points.round(4).tolist(),
            "representation": "native frame; display subsampling only; no deskew"}


def terrain_payload(msg, cfg):
    if (msg.schema_version != 2 or msg.profile != cfg["profile"] or
            msg.header.frame_id != cfg["frames"]["output"]):
        raise ValueError("terrain schema/profile/frame differs from active config")
    out = {"frame": msg.header.frame_id,
           "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
           "sequence": int(msg.sequence), "session": str(msg.session_id),
           "geometry_hash": msg.geometry_hash, "profile": msg.profile,
           "motion_compensated": bool(msg.motion_compensated),
           "operator_commissioned": bool(msg.operator_commissioned)}
    for name in ("height", "validity", "confidence", "variance", "age"):
        a = np.asarray(getattr(msg, name), dtype=float)
        if a.shape != (187,) or not np.isfinite(a).all():
            raise ValueError("malformed terrain " + name)
        out[name] = a.tolist()
    if (not np.isin(out["validity"], (0, 1)).all() or
            np.any(np.abs(out["height"]) > 1) or
            np.any(np.asarray(out["confidence"]) < 0) or
            np.any(np.asarray(out["confidence"]) > 1) or
            np.any(np.asarray(out["age"])[np.asarray(out["validity"]) == 1] < 0) or
            np.any(np.asarray(out["variance"]) < 0)):
        raise ValueError("terrain values outside contract")
    unknown = np.asarray(out["validity"]) == 0
    if np.any(np.asarray(out["height"])[unknown] != 0) or np.any(np.asarray(out["confidence"])[unknown] != 0):
        raise ValueError("unknown terrain must have zero height/confidence")
    return out


def collect(args):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from m20_heightmap_msgs.msg import TerrainObservation
    from m20_heightmap.cloud import read_cloud
    from m20_heightmap.runtime import load_config
    from m20_heightmap.geometry import Pose, PoseBuffer, rotation, yaw_rotation

    board = Path("/opt/robot/os_type").read_text().strip()
    if board != "CA9B_GOS":
        raise ValueError("run collector on GOS, observed board=" + board)
    cfg = load_config(args.config)
    try:
        front = front_position(cfg)
        geometry = {"front_lidar_base_m": front.tolist(), "source": "active runtime visibility/site config"}
    except ValueError as exc:
        front, geometry = None, {"error": str(exc)}
    poses = PoseBuffer(cfg["runtime"]["pose_buffer_size"], cfg["runtime"]["pose_max_gap_s"])
    memory = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        key, value = line.split(":", 1)
        memory[key] = int(value.split()[0]) * 1024
    # Two latest cloud buffers, NumPy working copies, and Python/ROS overhead.
    memory_budget = 4 * cfg["runtime"]["max_cloud_bytes"] + 256 * 1024**2
    available = memory["MemAvailable"]
    cpus = os.cpu_count() or len(os.sched_getaffinity(0))
    load = os.getloadavg()[0]
    if available < memory_budget or load >= cpus:
        raise ValueError("debug resource admission failed: memory/load has no measured headroom")
    started = time.monotonic()
    rclpy.init()
    node = rclpy.create_node("m20_readonly_lidar_debug_" + str(os.getpid()))
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    latest, counts, errors = {}, {}, {}
    topics = {"raw": cfg["topics"]["cloud"],
              "front": cfg["runtime"]["mapper_cloud_topic"],
              "terrain": cfg["topics"]["terrain"],
              "pose": cfg["topics"]["odom"],
              "mapper_status": cfg["topics"]["status"] + "/m20_heightmap_gos",
              "native_status": "/m20/gos/native_status"}

    def cache(key, msg):
        latest[key] = (msg, time.monotonic(), node.get_clock().now().nanoseconds * 1e-9)
        counts[key] = counts.get(key, 0) + 1

    def pose_message(msg):
        try:
            if msg.header.frame_id != cfg["frames"]["world"] or msg.child_frame_id != cfg["frames"]["base"]:
                raise ValueError("display pose frame differs from configured base/world")
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            poses.add(Pose(stamp, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w], np.zeros(6)))
            counts["pose"] = counts.get("pose", 0) + 1
            errors.pop("display_pose", None)
        except ValueError as exc:
            errors["display_pose"] = str(exc)

    subs = []
    subs.append(node.create_subscription(Odometry, topics["pose"], pose_message, qos))
    for key in ("raw", "front"):
        subs.append(node.create_subscription(PointCloud2, topics[key],
                    lambda m, k=key: cache(k, m), qos))
    subs.append(node.create_subscription(TerrainObservation, topics["terrain"],
                lambda m: cache("terrain", m), qos))
    for key in ("mapper_status", "native_status"):
        subs.append(node.create_subscription(String, topics[key],
                    lambda m, k=key: cache(k, m), qos))
    serial = Path("/sys/firmware/devicetree/base/serial-number")
    identity = {"board": board, "serial": serial.read_bytes().decode().strip("\0\n")
                if serial.exists() else "unavailable", "hostname": os.uname().nodename,
                "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
                "release": str(Path.cwd().resolve()),
                "config": str(Path(args.config).resolve())}
    limits = {"hz": args.hz, "max_points_per_cloud": args.max_points,
              "seconds": args.seconds, "max_cell_age_s": cfg["estimator"]["max_map_age_s"],
              "terrain_frame_age_s": cfg["aos"]["max_alignment_age_s"],
              "future_tolerance_s": cfg["runtime"]["future_tolerance_s"],
              "cloud_frame_age_s": cfg["runtime"]["cloud_input_timeout_s"],
              "scan_duration_s": cfg["runtime"]["scan_duration_s"],
              "resource_admission": {"available_bytes": available,
                  "calculated_memory_budget_bytes": memory_budget,
                  "cpu_count": cpus, "load_1m": load,
                  "collector_allowed_cpus": sorted(os.sched_getaffinity(0)),
                  "executor_threads": 1, "numpy_threads": os.environ.get("OPENBLAS_NUM_THREADS"),
                  "display_xyz_json_bytes_per_second_upper_estimate": args.max_points * 3 * 24 * 2 * args.hz}}
    next_emit = started
    try:
        while time.monotonic() - started < args.seconds:
            node_now = time.monotonic()
            rclpy.spin_once(node, timeout_sec=max(0., min(.02, next_emit - node_now)))
            now = time.monotonic()
            if now < next_emit:
                continue
            node_time = node.get_clock().now().nanoseconds * 1e-9
            next_emit = now + 1 / args.hz
            packet = {"schema": SCHEMA, "identity": identity, "topics": topics,
                      "geometry": geometry,
                      "limits": limits, "elapsed_s": now - started,
                      "board_now": node_time, "counts": dict(counts), "streams": {}}
            for key, (msg, received, received_ros) in list(latest.items()):
                try:
                    if key in ("raw", "front"):
                        payload = sample_cloud(msg, read_cloud, args.max_points, cfg)
                        if front is not None:
                            payload.update(cloud_origins(msg.header.frame_id, cfg, front, rotation))
                        # Header is scan start/end per configuration; do not call it newest-point age.
                    elif key == "terrain":
                        payload = terrain_payload(msg, cfg)
                        if front is not None:
                            try:
                                pose = poses.at(payload["stamp"])
                                payload["front_lidar_origin_m"] = terrain_origin(front, pose, yaw_rotation)
                                payload["origin_pose_stamp"] = pose.stamp
                            except ValueError as exc:
                                payload["front_origin_error"] = str(exc)
                    else:
                        payload = {"status": json.loads(msg.data)}
                    payload["silence_s"] = max(0., now - received)
                    payload["receive_ros"] = received_ros
                    if "stamp" in payload:
                        payload["header_age_s"] = node_time - payload["stamp"]
                    packet["streams"][key] = payload
                    errors.pop(key, None)
                except (ValueError, TypeError, KeyError) as exc:
                    errors[key] = str(exc)
            packet["errors"] = dict(errors)
            print(json.dumps(packet, ensure_ascii=False, allow_nan=False, separators=(",", ":")), flush=True)
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default="runtime/heightmap_v1.local.json")
    p.add_argument("--seconds", type=float, default=1800)
    p.add_argument("--hz", type=float, default=2)
    p.add_argument("--max-points", type=int, default=3000)
    args = p.parse_args()
    if not 1 <= args.seconds <= 3600 or not .2 <= args.hz <= 5 or not 100 <= args.max_points <= 10000:
        p.error("bounded debug budget: seconds 1..3600, hz .2..5, points 100..10000")
    collect(args)


if __name__ == "__main__":
    main()
