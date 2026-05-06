#!/usr/bin/env python3
# coding=utf-8

import copy
import math
import os
import threading
import time

LOCK = threading.Lock()

MAX_TRAJECTORY_SAMPLES = 5000
MAX_MISSION_HISTORY = 100
ENABLE_TRAJECTORY_METRICS = os.getenv("ENABLE_TRAJECTORY_METRICS", "0") == "1"
ENABLE_PATH_DEVIATION_METRICS = os.getenv("ENABLE_PATH_DEVIATION_METRICS", "0") == "1"
ENABLE_MISSION_POSE_TRACE = os.getenv("ENABLE_MISSION_POSE_TRACE", "0") == "1"


def _default_metrics():
    return {
        "run_started_at": time.time(),
        "trajectory": [],
        "distance": {
            "total_m": 0.0,
            "sample_count": 0,
            "last_pose": None,
            "last_update": None,
            "started_at": time.time(),
            "ignored_jump_count": 0,
        },
        "current_path": [],
        "path_stats": {
            "sample_count": 0,
            "mean_deviation_m": None,
            "max_deviation_m": None,
            "last_deviation_m": None,
            "active_path_points": 0,
            "active_path_length_m": 0.0,
        },
        "planner_stats": {
            "count": 0,
            "success_count": 0,
            "failure_count": 0,
            "replan_count": 0,
            "last_duration_ms": None,
            "mean_duration_ms": None,
            "max_duration_ms": None,
            "last_path_points": 0,
        },
        "missions": [],
        "current_mission": None,
    }


METRICS = _default_metrics()


def _cap_list(items, max_len):
    if len(items) > max_len:
        del items[:-max_len]


def _path_length(path_xy):
    total = 0.0
    for idx in range(1, len(path_xy)):
        x0, y0 = path_xy[idx - 1]
        x1, y1 = path_xy[idx]
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def _point_to_segment_distance(px, py, ax, ay, bx, by):
    dx = bx - ax
    dy = by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)

    t = ((px - ax) * dx + (py - ay) * dy) / seg_len_sq
    t = min(1.0, max(0.0, t))
    qx = ax + t * dx
    qy = ay + t * dy
    return math.hypot(px - qx, py - qy)


def _distance_to_path(path_xy, x, y):
    if len(path_xy) < 2:
        return None

    best = None
    for idx in range(1, len(path_xy)):
        ax, ay = path_xy[idx - 1]
        bx, by = path_xy[idx]
        dist = _point_to_segment_distance(x, y, ax, ay, bx, by)
        if best is None or dist < best:
            best = dist
    return best


def _mean(values):
    vals = [float(v) for v in values if v is not None]
    if not vals:
        return None
    return sum(vals) / len(vals)


def _normalize_status(status):
    mapping = {
        "completed": "success",
        "failed": "failed",
        "aborted": "aborted",
        "requested": "running",
        "accepted": "running",
    }
    return mapping.get(status, str(status))


def reset_metrics():
    global METRICS
    with LOCK:
        METRICS = _default_metrics()


def reset_distance():
    with LOCK:
        METRICS["distance"] = {
            "total_m": 0.0,
            "sample_count": 0,
            "last_pose": None,
            "last_update": None,
            "started_at": time.time(),
            "ignored_jump_count": 0,
        }


def get_distance_snapshot():
    with LOCK:
        distance = copy.deepcopy(METRICS["distance"])

    distance["total_cm"] = distance["total_m"] * 100.0
    distance["total_km"] = distance["total_m"] / 1000.0
    distance["duration_sec"] = max(0.0, time.time() - distance["started_at"])
    return distance


def set_active_path(path_xy):
    normalized = [(float(x), float(y)) for (x, y) in (path_xy or [])]
    with LOCK:
        METRICS["current_path"] = normalized
        METRICS["path_stats"]["active_path_points"] = len(normalized)
        METRICS["path_stats"]["active_path_length_m"] = _path_length(normalized)


def clear_active_path():
    with LOCK:
        METRICS["current_path"] = []
        METRICS["path_stats"]["active_path_points"] = 0
        METRICS["path_stats"]["active_path_length_m"] = 0.0
        METRICS["path_stats"]["last_deviation_m"] = None


def record_planner_result(duration_sec, path_xy, success, is_replan=False):
    duration_ms = max(0.0, float(duration_sec) * 1000.0)
    with LOCK:
        stats = METRICS["planner_stats"]
        stats["count"] += 1
        if is_replan:
            stats["replan_count"] += 1
        stats["last_duration_ms"] = duration_ms
        stats["last_path_points"] = len(path_xy or [])
        if success:
            stats["success_count"] += 1
        else:
            stats["failure_count"] += 1
        prev_mean = stats["mean_duration_ms"]
        if prev_mean is None:
            stats["mean_duration_ms"] = duration_ms
        else:
            count = stats["count"]
            stats["mean_duration_ms"] = ((prev_mean * (count - 1)) + duration_ms) / count
        prev_max = stats["max_duration_ms"]
        stats["max_duration_ms"] = duration_ms if prev_max is None else max(prev_max, duration_ms)


def start_mission(goal, planned_path=None):
    started_at = time.time()
    mission = {
        "mission_id": f"m{int(started_at * 1000)}",
        "start_time": started_at,
        "end_time": None,
        "duration_sec": None,
        "status": "running",
        "goal_reached": None,
        "semantic_goal_verified": None,
        "intervention_count": 0,
        "replan_count": 0,
        "goal": {
            "x": float(goal["x"]),
            "y": float(goal["y"]),
            "yaw": float(goal.get("yaw", 0.0)),
        },
        "path_length_planned_m": _path_length(planned_path or []),
        "path_length_executed_m": 0.0,
        "mean_path_deviation_m": None,
        "max_path_deviation_m": None,
        "final_goal_error_m": None,
        "samples": 0,
        "result": None,
        "pose_trace": [],
        "distance_to_goal_m": None,
    }
    with LOCK:
        METRICS["current_mission"] = mission
    return mission["mission_id"]


def record_intervention(reason=None):
    with LOCK:
        mission = METRICS["current_mission"]
        if mission is None:
            return
        mission["intervention_count"] += 1
        if reason is not None:
            mission["result"] = str(reason)


def record_replan():
    with LOCK:
        mission = METRICS["current_mission"]
        if mission is None:
            return
        mission["replan_count"] += 1


def update_current_mission(status=None, result=None):
    with LOCK:
        mission = METRICS["current_mission"]
        if mission is None:
            return
        if status is not None:
            mission["status"] = str(status)
        if result is not None:
            mission["result"] = str(result)


def finalize_current_mission(status, result=None, pose=None):
    with LOCK:
        mission = METRICS["current_mission"]
        if mission is None:
            return

        finished_at = time.time()
        mission["end_time"] = finished_at
        mission["duration_sec"] = max(0.0, finished_at - mission["start_time"])
        mission["status"] = _normalize_status(status)
        if result is not None:
            mission["result"] = str(result)
        if pose is not None:
            px = float(pose["x"])
            py = float(pose["y"])
            mission["final_goal_error_m"] = math.hypot(
                px - mission["goal"]["x"],
                py - mission["goal"]["y"],
            )
            mission["distance_to_goal_m"] = mission["final_goal_error_m"]
        mission["goal_reached"] = mission["status"] == "success"
        METRICS["missions"].append(copy.deepcopy(mission))
        _cap_list(METRICS["missions"], MAX_MISSION_HISTORY)
        METRICS["current_mission"] = None


def record_pose_sample(ts, x, y, theta, ok):
    sample = {
        "t": float(ts),
        "x": float(x),
        "y": float(y),
        "theta": float(theta),
        "ok": bool(ok),
    }
    with LOCK:
        if ENABLE_TRAJECTORY_METRICS:
            METRICS["trajectory"].append(sample)
            _cap_list(METRICS["trajectory"], MAX_TRAJECTORY_SAMPLES)

        distance = METRICS["distance"]
        if ok:
            last_pose = distance["last_pose"]
            if last_pose is not None:
                step = math.hypot(sample["x"] - last_pose["x"], sample["y"] - last_pose["y"])
                if 0.005 <= step <= 2.0:
                    distance["total_m"] += step
                elif step > 2.0:
                    distance["ignored_jump_count"] += 1
            distance["sample_count"] += 1
            distance["last_pose"] = {
                "t": sample["t"],
                "x": sample["x"],
                "y": sample["y"],
                "theta": sample["theta"],
            }
            distance["last_update"] = sample["t"]

        current_path = METRICS["current_path"]
        mission = METRICS["current_mission"]
        deviation = None
        if ENABLE_PATH_DEVIATION_METRICS and ok and len(current_path) >= 2:
            deviation = _distance_to_path(current_path, sample["x"], sample["y"])
            if deviation is not None:
                stats = METRICS["path_stats"]
                stats["sample_count"] += 1
                stats["last_deviation_m"] = deviation
                prev_mean = stats["mean_deviation_m"]
                if prev_mean is None:
                    stats["mean_deviation_m"] = deviation
                else:
                    n = stats["sample_count"]
                    stats["mean_deviation_m"] = ((prev_mean * (n - 1)) + deviation) / n
                prev_max = stats["max_deviation_m"]
                stats["max_deviation_m"] = deviation if prev_max is None else max(prev_max, deviation)

        if mission is not None and ok:
            if ENABLE_MISSION_POSE_TRACE and mission["pose_trace"]:
                prev = mission["pose_trace"][-1]
                mission["path_length_executed_m"] += math.hypot(sample["x"] - prev["x"], sample["y"] - prev["y"])
            mission["samples"] += 1
            if ENABLE_MISSION_POSE_TRACE:
                mission["pose_trace"].append(sample)
                _cap_list(mission["pose_trace"], 500)
            mission["distance_to_goal_m"] = math.hypot(
                sample["x"] - mission["goal"]["x"],
                sample["y"] - mission["goal"]["y"],
            )
            if deviation is not None:
                prev_mean = mission["mean_path_deviation_m"]
                if prev_mean is None:
                    mission["mean_path_deviation_m"] = deviation
                else:
                    n = mission["samples"]
                    mission["mean_path_deviation_m"] = ((prev_mean * (n - 1)) + deviation) / n
                prev_max = mission["max_path_deviation_m"]
                mission["max_path_deviation_m"] = deviation if prev_max is None else max(prev_max, deviation)


def _build_summary(snapshot):
    missions = snapshot["missions"]
    success_missions = [m for m in missions if m["status"] == "success"]
    return {
        "success_rate": (len(success_missions) / len(missions)) if missions else None,
        "mean_time_to_goal_sec": _mean(m.get("duration_sec") for m in success_missions),
        "mean_path_deviation_m": _mean(m.get("mean_path_deviation_m") for m in missions),
        "mean_intervention_count": _mean(m.get("intervention_count") for m in missions),
        "trajectory_samples": len(snapshot["trajectory"]),
        "mission_count": len(missions),
        "completed_missions": len(success_missions),
        "failed_or_aborted_missions": len([m for m in missions if m["status"] in ("failed", "aborted")]),
        "run_duration_sec": max(0.0, time.time() - snapshot["run_started_at"]),
    }


def _build_summary_light(run_started_at, trajectory_count, missions):
    success_missions = [m for m in missions if m["status"] == "success"]
    return {
        "success_rate": (len(success_missions) / len(missions)) if missions else None,
        "mean_time_to_goal_sec": _mean(m.get("duration_sec") for m in success_missions),
        "mean_path_deviation_m": _mean(m.get("mean_path_deviation_m") for m in missions),
        "mean_intervention_count": _mean(m.get("intervention_count") for m in missions),
        "trajectory_samples": trajectory_count,
        "mission_count": len(missions),
        "completed_missions": len(success_missions),
        "failed_or_aborted_missions": len([m for m in missions if m["status"] in ("failed", "aborted")]),
        "run_duration_sec": max(0.0, time.time() - run_started_at),
    }


def _strip_pose_trace(mission):
    if mission is None:
        return None
    item = copy.deepcopy(mission)
    item.pop("pose_trace", None)
    return item


def _field_status_map(snapshot):
    has_missions = bool(snapshot["missions"])
    return {
        "missions.duration_sec": "real" if has_missions else "unsupported",
        "missions.intervention_count": "real" if has_missions else "unsupported",
        "missions.mean_path_deviation_m": "derived" if has_missions else "unsupported",
        "missions.max_path_deviation_m": "derived" if has_missions else "unsupported",
        "missions.final_goal_error_m": "derived" if has_missions else "unsupported",
        "missions.path_length_planned_m": "derived" if has_missions else "unsupported",
        "missions.path_length_executed_m": "derived" if has_missions else "unsupported",
        "missions.goal_reached": "derived" if has_missions else "unsupported",
        "missions.semantic_goal_verified": "unsupported",
        "planner_stats.replan_count": "real",
        "summary.success_rate": "derived",
        "summary.mean_time_to_goal_sec": "derived",
        "summary.mean_path_deviation_m": "derived",
        "summary.mean_intervention_count": "derived",
    }


def get_metrics_snapshot(
    include_trajectory=False,
):
    if not include_trajectory:
        with LOCK:
            run_started_at = METRICS["run_started_at"]
            trajectory_count = len(METRICS["trajectory"])
            missions = [_strip_pose_trace(mission) for mission in METRICS["missions"]]
            current_mission = _strip_pose_trace(METRICS["current_mission"])
            planner_stats = copy.deepcopy(METRICS["planner_stats"])

        summary = _build_summary_light(run_started_at, trajectory_count, missions)
        snapshot = {
            "run_started_at": run_started_at,
            "missions": missions,
            "current_mission": current_mission,
            "planner_stats": planner_stats,
            "summary": summary,
            "payload_mode": {
                "trajectory": False,
            },
        }
        snapshot["provenance"] = {
            "mode": "light",
            "field_status": _field_status_map(snapshot),
            "note": "Light payload avoids trajectory and pose_trace copies on the robot.",
        }
        return snapshot

    with LOCK:
        snapshot = copy.deepcopy(METRICS)

    snapshot["summary"] = _build_summary(snapshot)
    snapshot["provenance"] = {
        "field_status": _field_status_map(snapshot),
        "legend": {
            "real": "measured directly or explicitly provided",
            "derived": "computed from measured data",
            "unsupported": "not available in the current robot-side stack",
        },
    }

    for mission in snapshot["missions"]:
        mission.pop("pose_trace", None)
    if snapshot["current_mission"] is not None:
        snapshot["current_mission"].pop("pose_trace", None)

    if not include_trajectory:
        snapshot.pop("trajectory", None)
        snapshot.pop("current_path", None)

    snapshot["payload_mode"] = {
        "trajectory": bool(include_trajectory),
    }
    return snapshot
