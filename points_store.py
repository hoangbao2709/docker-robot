#!/usr/bin/env python3
# coding=utf-8

import json
import os
import threading

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
POINTS_FILE = os.path.join(BASE_DIR, "named_points.json")

_LOCK = threading.Lock()


def _ensure_file():
    if not os.path.isfile(POINTS_FILE):
        with open(POINTS_FILE, "w", encoding="utf-8") as f:
            json.dump({}, f, ensure_ascii=False, indent=2)


def load_points():
    _ensure_file()
    with _LOCK:
        with open(POINTS_FILE, "r", encoding="utf-8") as f:
            return json.load(f)


def save_points(points):
    with _LOCK:
        with open(POINTS_FILE, "w", encoding="utf-8") as f:
            json.dump(points, f, ensure_ascii=False, indent=2)


def upsert_point(name: str, x: float, y: float, yaw: float, **metadata):
    points = load_points()
    point = {
        "x": float(x),
        "y": float(y),
        "yaw": float(yaw),
    }
    for key, value in metadata.items():
        if value is None:
            continue
        point[key] = value
    points[name] = point
    save_points(points)
    return point


def get_point(name: str):
    points = load_points()
    return points.get(name)


def delete_point(name: str):
    points = load_points()
    if name in points:
        deleted = points.pop(name)
        save_points(points)
        return deleted
    return None
