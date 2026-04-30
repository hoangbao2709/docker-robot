#!/usr/bin/env python3
# coding=utf-8

import copy
import threading
import time

LOCK = threading.Lock()

GOAL_REQUEST = None
INITIAL_POSE_REQUEST = None
CLEAR_REQUEST = False
SAVE_REQUEST_NAME = None
MAP_OVERRIDE = None
RAW_MAP = None

SHARED_STATE = {
    "map_version": 0,
    "map_info": None,
    "render_info": None,
    "pose": {
        "x": 0.0,
        "y": 0.0,
        "theta": 0.0,
        "stamp": 0.0,
        "ok": False,
    },
    "goal": {
        "x": None,
        "y": None,
        "yaw": None,
        "stamp": 0.0,
    },
    "paths": {
        # "a_star": [],
        "plan": [],
        "local_plan": [],
        "received_plan": [],
    },
    "scan": {
        "points": [],
        "raw": {
            "samples": [],
            "range_min": 0.0,
            "range_max": 0.0,
        },
        "transform": {
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
        },
        "stamp": 0.0,
        "frame_id": "",
        "ok": False,
    },
    "status": {
        "slam_ok": False,
        "tf_ok": False,
        "planner_ok": False,
        "planner_msg": "idle",
        "map_age_sec": None,
        "pose_age_sec": None,
        "last_update": 0.0,
    },
}


def set_goal_request(x, y, yaw, u=None, v=None):
    global GOAL_REQUEST
    with LOCK:
        GOAL_REQUEST = {
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
        }
        if u is not None:
            GOAL_REQUEST["u"] = float(u)
        if v is not None:
            GOAL_REQUEST["v"] = float(v)


def pop_goal_request():
    global GOAL_REQUEST
    with LOCK:
        req = GOAL_REQUEST
        GOAL_REQUEST = None
    return req


def set_initial_pose_request(x, y, yaw, u=None, v=None):
    global INITIAL_POSE_REQUEST
    with LOCK:
        INITIAL_POSE_REQUEST = {
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
        }
        if u is not None:
            INITIAL_POSE_REQUEST["u"] = float(u)
        if v is not None:
            INITIAL_POSE_REQUEST["v"] = float(v)


def pop_initial_pose_request():
    global INITIAL_POSE_REQUEST
    with LOCK:
        req = INITIAL_POSE_REQUEST
        INITIAL_POSE_REQUEST = None
    return req


def request_clear_path():
    global CLEAR_REQUEST
    with LOCK:
        CLEAR_REQUEST = True


def pop_clear_request():
    global CLEAR_REQUEST
    with LOCK:
        flag = CLEAR_REQUEST
        CLEAR_REQUEST = False
    return flag


def set_save_request(name: str):
    global SAVE_REQUEST_NAME
    with LOCK:
        SAVE_REQUEST_NAME = name


def pop_save_request():
    global SAVE_REQUEST_NAME
    with LOCK:
        name = SAVE_REQUEST_NAME
        SAVE_REQUEST_NAME = None
    return name


def set_map_override(image_path, map_info, render_info):
    global MAP_OVERRIDE
    with LOCK:
        MAP_OVERRIDE = {
            "image_path": str(image_path),
            "map_info": copy.deepcopy(map_info),
            "render_info": copy.deepcopy(render_info),
            "map_version": int(time.time() * 1000),
        }


def get_map_override():
    with LOCK:
        if MAP_OVERRIDE is None:
            return None
        return copy.deepcopy(MAP_OVERRIDE)


def clear_map_override():
    global MAP_OVERRIDE
    with LOCK:
        MAP_OVERRIDE = None


def set_raw_map_snapshot(
    *,
    width,
    height,
    resolution,
    origin_x,
    origin_y,
    frame_id,
    data,
    map_version,
):
    global RAW_MAP
    with LOCK:
        RAW_MAP = {
            "width": int(width),
            "height": int(height),
            "resolution": float(resolution),
            "origin_x": float(origin_x),
            "origin_y": float(origin_y),
            "frame_id": str(frame_id or "map"),
            "data": bytes(data),
            "map_version": int(map_version),
            "stamp": time.time(),
            "dtype": "int8",
            "encoding": "occupancy-grid-int8-row-major",
        }


def get_raw_map_snapshot():
    with LOCK:
        if RAW_MAP is None:
            return None
        snapshot = dict(RAW_MAP)
        snapshot["data"] = bytes(RAW_MAP["data"])
        return snapshot


def get_state_snapshot():
    with LOCK:
        return copy.deepcopy(SHARED_STATE)


def get_state_light_snapshot():
    with LOCK:
        scan = SHARED_STATE["scan"]
        return {
            "map_version": SHARED_STATE["map_version"],
            "map_info": copy.deepcopy(SHARED_STATE["map_info"]),
            "render_info": copy.deepcopy(SHARED_STATE["render_info"]),
            "pose": copy.deepcopy(SHARED_STATE["pose"]),
            "goal": copy.deepcopy(SHARED_STATE["goal"]),
            "paths": copy.deepcopy(SHARED_STATE["paths"]),
            "scan": {
                "points": [],
                "raw": copy.deepcopy(scan.get("raw") or {}),
                "transform": copy.deepcopy(scan.get("transform") or {}),
                "stamp": scan.get("stamp", 0.0),
                "frame_id": scan.get("frame_id", ""),
                "ok": scan.get("ok", False),
            },
            "status": copy.deepcopy(SHARED_STATE["status"]),
        }


def update_shared_state(fn):
    with LOCK:
        fn(SHARED_STATE)

