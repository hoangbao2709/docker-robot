#!/usr/bin/env python3
# coding=utf-8

import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_PNG_PATH = os.path.join(BASE_DIR, "map.png")
MAP_SAVE_DIR = os.path.join(BASE_DIR, "saved_maps")

os.makedirs(MAP_SAVE_DIR, exist_ok=True)
# ==== ROS / Navigation / Cartographer shared config ====
ROS_WS = "/root/yahboomcar_ws"
CARTOGRAPHER_PKG = "mi_cartographer"
CARTOGRAPHER_LAUNCH = "cartographer.launch.py"
CARTOGRAPHER_CONFIG_DIR = os.path.join(
    ROS_WS, "src", "mi_cartographer", "config"
)

# file lua mặc định
DEFAULT_LUA_BASENAME = "xgo_2d.lua"

# frames / topics khớp với Nav2
MAP_FRAME = "map"
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
SCAN_TOPIC = "/scan"
ODOM_TOPIC = "/odom"
CMD_VEL_TOPIC = "/cmd_vel"

USE_SIM_TIME = False

SLAM_LOG_PATH = "/tmp/mi_cartographer_web.log"
