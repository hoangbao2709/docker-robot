#!/usr/bin/env python3
# coding=utf-8

import copy
import math
import threading
import time

LOCK = threading.Lock()

MAX_TRAJECTORY_SAMPLES = 5000
MAX_MISSION_HISTORY = 100

METRICS = {
    "run_started_at": time.time(),
    "trajectory": [],
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
        "last_duration_ms": None,
        "mean_duration_ms": None,
        "max_duration_ms": None,
        "last_path_points": 0,
    },
    "missions": [],
    "current_mission": None,
    "reference": {
        "label": None,
        "trajectory": [],
        "loaded_at": None,
    },
}


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


def reset_metrics():
    with LOCK:
        METRICS["run_started_at"] = time.time()
        METRICS["trajectory"] = []
        METRICS["current_path"] = []
        METRICS["path_stats"] = {
            "sample_count": 0,
            "mean_deviation_m": None,
            "max_deviation_m": None,
            "last_deviation_m": None,
            "active_path_points": 0,
            "active_path_length_m": 0.0,
        }
        METRICS["planner_stats"] = {
            "count": 0,
            "success_count": 0,
            "failure_count": 0,
            "last_duration_ms": None,
            "mean_duration_ms": None,
            "max_duration_ms": None,
            "last_path_points": 0,
        }
        METRICS["missions"] = []
        METRICS["current_mission"] = None
        METRICS["reference"] = {
            "label": None,
            "trajectory": [],
            "loaded_at": None,
        }


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


def record_planner_result(duration_sec, path_xy, success):
    duration_ms = max(0.0, float(duration_sec) * 1000.0)
    with LOCK:
        stats = METRICS["planner_stats"]
        stats["count"] += 1
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
        "id": int(started_at * 1000),
        "status": "requested",
        "requested_at": started_at,
        "accepted_at": None,
        "finished_at": None,
        "duration_sec": None,
        "goal": {
            "x": float(goal["x"]),
            "y": float(goal["y"]),
            "yaw": float(goal.get("yaw", 0.0)),
        },
        "planned_path_points": len(planned_path or []),
        "planned_path_length_m": _path_length(planned_path or []),
        "result": None,
        "samples": 0,
        "pose_trace": [],
        "distance_to_goal_m": None,
    }
    with LOCK:
        METRICS["current_mission"] = mission
    return mission["id"]


def update_current_mission(status=None, result=None):
    with LOCK:
        mission = METRICS["current_mission"]
        if mission is None:
            return
        if status is not None:
            mission["status"] = str(status)
            if status == "accepted" and mission["accepted_at"] is None:
                mission["accepted_at"] = time.time()
        if result is not None:
            mission["result"] = str(result)


def finalize_current_mission(status, result=None, pose=None):
    with LOCK:
        mission = METRICS["current_mission"]
        if mission is None:
            return

        finished_at = time.time()
        mission["status"] = str(status)
        mission["finished_at"] = finished_at
        mission["duration_sec"] = max(0.0, finished_at - mission["requested_at"])
        if result is not None:
            mission["result"] = str(result)
        if pose is not None:
            px = float(pose["x"])
            py = float(pose["y"])
            mission["distance_to_goal_m"] = math.hypot(
                px - mission["goal"]["x"],
                py - mission["goal"]["y"],
            )
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
        METRICS["trajectory"].append(sample)
        _cap_list(METRICS["trajectory"], MAX_TRAJECTORY_SAMPLES)

        current_path = METRICS["current_path"]
        if ok and len(current_path) >= 2:
            dist = _distance_to_path(current_path, sample["x"], sample["y"])
            if dist is not None:
                stats = METRICS["path_stats"]
                stats["sample_count"] += 1
                stats["last_deviation_m"] = dist
                prev_mean = stats["mean_deviation_m"]
                if prev_mean is None:
                    stats["mean_deviation_m"] = dist
                else:
                    n = stats["sample_count"]
                    stats["mean_deviation_m"] = ((prev_mean * (n - 1)) + dist) / n
                prev_max = stats["max_deviation_m"]
                stats["max_deviation_m"] = dist if prev_max is None else max(prev_max, dist)

        mission = METRICS["current_mission"]
        if mission is not None and ok:
            mission["samples"] += 1
            mission["pose_trace"].append(sample)
            _cap_list(mission["pose_trace"], 500)
            mission["distance_to_goal_m"] = math.hypot(
                sample["x"] - mission["goal"]["x"],
                sample["y"] - mission["goal"]["y"],
            )


def set_reference_trajectory(samples, label=None):
    normalized = []
    for item in samples:
        normalized.append({
            "t": float(item["t"]),
            "x": float(item["x"]),
            "y": float(item["y"]),
            "theta": float(item.get("theta", 0.0)),
        })
    normalized.sort(key=lambda item: item["t"])
    with LOCK:
        METRICS["reference"] = {
            "label": label or "reference",
            "trajectory": normalized,
            "loaded_at": time.time(),
        }


def clear_reference_trajectory():
    with LOCK:
        METRICS["reference"] = {
            "label": None,
            "trajectory": [],
            "loaded_at": None,
        }


def _nearest_reference_sample(ref_samples, ts):
    if not ref_samples:
        return None
    best = ref_samples[0]
    best_dt = abs(best["t"] - ts)
    for item in ref_samples[1:]:
        dt = abs(item["t"] - ts)
        if dt < best_dt:
            best = item
            best_dt = dt
    return best


def _compute_reference_metrics(trajectory, ref_samples):
    if not trajectory or not ref_samples:
        return None

    pos_errors = []
    heading_errors = []
    for sample in trajectory:
        if not sample["ok"]:
            continue
        ref = _nearest_reference_sample(ref_samples, sample["t"])
        if ref is None:
            continue
        pos_errors.append(math.hypot(sample["x"] - ref["x"], sample["y"] - ref["y"]))
        heading_errors.append(abs(sample["theta"] - ref["theta"]))

    if not pos_errors:
        return None

    rmse = math.sqrt(sum(err * err for err in pos_errors) / len(pos_errors))
    mean_abs = sum(pos_errors) / len(pos_errors)
    final_drift = pos_errors[-1]
    mean_heading = sum(heading_errors) / len(heading_errors) if heading_errors else None
    return {
        "sample_count": len(pos_errors),
        "rmse_position_m": rmse,
        "mean_position_error_m": mean_abs,
        "final_drift_m": final_drift,
        "mean_heading_error_rad": mean_heading,
    }


def get_metrics_snapshot():
    with LOCK:
        snapshot = copy.deepcopy(METRICS)

    missions = snapshot["missions"]
    completed = [m for m in missions if m["status"] == "completed"]
    failed = [m for m in missions if m["status"] not in ("completed",)]
    summary = {
        "trajectory_samples": len(snapshot["trajectory"]),
        "mission_count": len(missions),
        "completed_missions": len(completed),
        "failed_or_aborted_missions": len(failed),
        "success_rate": (len(completed) / len(missions)) if missions else None,
        "mean_time_to_goal_sec": (
            sum(m["duration_sec"] for m in completed if m["duration_sec"] is not None) / len(completed)
        ) if completed else None,
    }
    snapshot["summary"] = summary
    snapshot["reference_metrics"] = _compute_reference_metrics(
        snapshot["trajectory"],
        snapshot["reference"]["trajectory"],
    )
    return snapshot
