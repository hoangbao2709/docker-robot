#!/usr/bin/env python3
# coding=utf-8

import os
import shlex
import shutil
import subprocess
from datetime import datetime

LUA_PATH = "/root/yahboomcar_ws/src/mi_cartographer/config/dogzilla_base.lua"

def _build_restart_cmd(load_state_filename=None, load_frozen_state=True):
    load_args = ""
    if load_state_filename:
        load_args = (
            " \\\n  load_state_filename:=" + shlex.quote(load_state_filename) +
            " \\\n  load_frozen_state:=" + ("true" if load_frozen_state else "false")
        )

    return r"""
bash -lc '
pkill -f cartographer_node || true
pkill -f occupancy_grid_node || true
pkill -f "mi_cartographer cartographer.launch.py" || true
sleep 1
source /root/yahboomcar_ws/install/setup.bash
nohup ros2 launch mi_cartographer cartographer.launch.py \
  cartographer_config_dir:=/root/yahboomcar_ws/src/mi_cartographer/config \
  configuration_basename:=dogzilla_base.lua \
  launch_rviz:=false""" + load_args + r""" \
  >/tmp/mi_cartographer_web.log 2>&1 &
'
"""


def get_lua_path():
    return LUA_PATH


def read_lua_config():
    with open(LUA_PATH, "r", encoding="utf-8") as f:
        return f.read()


def backup_lua_config():
    if not os.path.isfile(LUA_PATH):
        raise FileNotFoundError(f"Lua file not found: {LUA_PATH}")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_path = f"{LUA_PATH}.{ts}.bak"
    shutil.copy2(LUA_PATH, backup_path)

    latest_backup = f"{LUA_PATH}.bak"
    shutil.copy2(LUA_PATH, latest_backup)

    return backup_path


def write_lua_config(text: str):
    backup_path = backup_lua_config()

    tmp_path = LUA_PATH + ".tmp"
    with open(tmp_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    os.replace(tmp_path, LUA_PATH)

    return backup_path


def restore_latest_backup():
    latest_backup = f"{LUA_PATH}.bak"
    if not os.path.isfile(latest_backup):
        raise FileNotFoundError(f"Backup file not found: {latest_backup}")
    shutil.copy2(latest_backup, LUA_PATH)
    return latest_backup


def restart_slam(load_state_filename=None, load_frozen_state=True):
    cmd = _build_restart_cmd(load_state_filename, load_frozen_state)
    result = subprocess.run(
        cmd,
        shell=True,
        executable="/bin/bash",
        capture_output=True,
        text=True,
    )
    return {
        "ok": result.returncode == 0,
        "returncode": result.returncode,
        "stdout": result.stdout,
        "stderr": result.stderr,
        "log_path": "/tmp/mi_cartographer_web.log",
        "load_state_filename": load_state_filename,
        "load_frozen_state": bool(load_frozen_state),
    }


def get_slam_status():
    try:
        result = subprocess.run(
            r"bash -lc 'pgrep -af cartographer'",
            shell=True,
            executable="/bin/bash",
            capture_output=True,
            text=True,
        )
        lines = [ln.strip() for ln in result.stdout.splitlines() if ln.strip()]
        return {
            "running": len(lines) > 0,
            "processes": lines,
            "lua_path": LUA_PATH,
            "log_path": "/tmp/mi_cartographer_web.log",
        }
    except Exception as e:
        return {
            "running": False,
            "processes": [],
            "lua_path": LUA_PATH,
            "log_path": "/tmp/mi_cartographer_web.log",
            "error": str(e),
        }

