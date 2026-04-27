#!/usr/bin/env python3
# coding=utf-8

import math
import time


def now_sec():
    return time.time()


def quat_to_yaw(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def yaw_to_quat(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return qz, qw
