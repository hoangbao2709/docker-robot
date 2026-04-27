#!/usr/bin/env python3
# coding=utf-8

"""
slam_live_map_mimi_gd2.py

Lightweight web map viewer for Raspberry Pi

UI:
- VIEW:
    + save map
    + load map
    + reset view
    + checkbox: robot, path, grid
- NAV:
    + stop & clear path
    + system status
    + coordinate
    + checkbox: robot, path, grid
    + chi NAV moi duoc click chon diem dich (2 step)

Display:
- map duoc render thanh anh VUONG
- map that nam giua
- vung ngoai map / chua quet hien mau xam
- overlay robot / path / grid ve bang canvas
- sua loi click goal khi zoom / pan
"""

import copy
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs
import cgi

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from rclpy.duration import Duration
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import NavigateToPose

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt



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


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_PNG_PATH = os.path.join(BASE_DIR, "map.png")
MAP_SAVE_DIR = os.path.join(BASE_DIR, "saved_maps")
os.makedirs(MAP_SAVE_DIR, exist_ok=True)

LOCK = threading.Lock()

GOAL_REQUEST = None
CLEAR_REQUEST = False
SAVE_REQUEST_NAME = None

SHARED_STATE = {
    "map_version": 0,
    "map_info": None,      # map that
    "render_info": None,   # khung vuong da pad
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
        "a_star": [],
    },

    "status": {
        "slam_ok": False,
        "tf_ok": False,
        "planner_ok": False,
        "planner_msg": "idle",
        "nav2_server_ok": False,
        "nav2_goal_active": False,
        "nav2_result": "idle",
        "map_age_sec": None,
        "pose_age_sec": None,
        "last_update": 0.0,
    },

}


def set_goal_request(u, v, yaw):
    global GOAL_REQUEST
    with LOCK:
        GOAL_REQUEST = {
            "u": float(u),
            "v": float(v),
            "yaw": float(yaw),
        }


def pop_goal_request():
    global GOAL_REQUEST
    with LOCK:
        req = GOAL_REQUEST
        GOAL_REQUEST = None
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


def get_state_snapshot():
    with LOCK:
        return copy.deepcopy(SHARED_STATE)


def update_shared_state(fn):
    with LOCK:
        fn(SHARED_STATE)


class ImageServer(BaseHTTPRequestHandler):
    server_version = "LightMapHTTP/1.0"

    def _send_bytes(self, status_code, content_type, data: bytes, extra_headers=None):
        self.send_response(status_code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        if extra_headers:
            for k, v in extra_headers.items():
                self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def _send_text(self, status_code, text):
        self._send_bytes(status_code, "text/plain; charset=utf-8", text.encode("utf-8"))

    def _send_json(self, status_code, obj):
        self._send_bytes(
            status_code,
            "application/json; charset=utf-8",
            json.dumps(obj, ensure_ascii=False).encode("utf-8"),
        )

    def _send_html(self, html):
        self._send_bytes(200, "text/html; charset=utf-8", html.encode("utf-8"))

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/" or path == "/index.html":
            self._send_html(self.build_index_html())
            return

        if path == "/state":
            self._send_json(200, get_state_snapshot())
            return

        if path == "/map.png":
            try:
                with open(MAP_PNG_PATH, "rb") as f:
                    data = f.read()
                self._send_bytes(
                    200,
                    "image/png",
                    data,
                    extra_headers={"Cache-Control": "no-store, no-cache, must-revalidate, max-age=0"},
                )
            except Exception:
                self.send_error(404, "map.png not found")
            return

        if path.startswith("/set_goal_pose"):
            try:
                qs = parse_qs(parsed.query)
                u = float(qs.get("u", [None])[0])
                v = float(qs.get("v", [None])[0])
                yaw = float(qs.get("yaw", [0.0])[0])

                if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
                    raise ValueError("u,v out of range")

                set_goal_request(u, v, yaw)
                self._send_text(200, "OK")
            except Exception as e:
                self.send_error(400, f"bad params: {e}")
            return

        if path.startswith("/clear_path"):
            request_clear_path()
            self._send_text(200, "CLEARED")
            return

        if path.startswith("/save_map"):
            try:
                qs = parse_qs(parsed.query)
                name = qs.get("name", [None])[0]
                if not name:
                    raise ValueError("name missing")
                safe = "".join(ch if ch.isalnum() or ch in "_-" else "_" for ch in name)
                if not safe:
                    raise ValueError("invalid name")
                set_save_request(safe)
                self._send_text(200, "SAVE_REQUESTED")
            except Exception as e:
                self.send_error(400, f"bad params: {e}")
            return

        if path.startswith("/maps/"):
            rel = path[len("/maps/"):]
            fname = os.path.basename(rel)
            fpath = os.path.join(MAP_SAVE_DIR, fname)
            if not os.path.isfile(fpath):
                self.send_error(404, "map file not found")
                return

            if fname.endswith(".yaml") or fname.endswith(".yml"):
                ctype = "text/yaml"
            elif fname.endswith(".pgm"):
                ctype = "image/x-portable-graymap"
            else:
                ctype = "application/octet-stream"

            with open(fpath, "rb") as f:
                data = f.read()

            self._send_bytes(
                200,
                ctype,
                data,
                extra_headers={"Content-Disposition": f'attachment; filename="{fname}"'},
            )
            return

        self.send_error(404)

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/upload_map":
            try:
                ctype, pdict = cgi.parse_header(self.headers.get("Content-Type"))
                if ctype != "multipart/form-data":
                    self.send_error(400, "Expected multipart/form-data")
                    return

                pdict["boundary"] = bytes(pdict["boundary"], "utf-8")
                pdict["CONTENT-LENGTH"] = int(self.headers.get("Content-Length", 0))

                form = cgi.FieldStorage(
                    fp=self.rfile,
                    headers=self.headers,
                    environ={
                        "REQUEST_METHOD": "POST",
                        "CONTENT_TYPE": self.headers.get("Content-Type"),
                    },
                )

                if "file" not in form:
                    self.send_error(400, "No file field")
                    return

                field_item = form["file"]
                if not field_item.filename:
                    self.send_error(400, "Empty filename")
                    return

                fname = os.path.basename(field_item.filename)
                safe = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in fname)
                ext = os.path.splitext(safe)[1].lower()

                if ext not in (".yaml", ".yml", ".pgm", ".png", ".jpg", ".jpeg"):
                    self.send_error(400, "Unsupported file type")
                    return

                fpath = os.path.join(MAP_SAVE_DIR, safe)
                with open(fpath, "wb") as f:
                    f.write(field_item.file.read())

                self._send_text(200, f"UPLOADED {safe}")
            except Exception as e:
                self.send_error(500, f"Upload error: {e}")
            return

        self.send_error(404)

    def log_message(self, fmt, *args):
        return

    def build_index_html(self):
        return """
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>SLAM Live Map</title>
  <style>
    html, body {
      margin: 0;
      padding: 0;
      background: #0b0b0b;
      color: #fff;
      font-family: Arial, sans-serif;
      height: 100%;
      overflow: hidden;
    }
    .wrap {
      position: relative;
      width: 100%;
      height: 100%;
      background: #111;
      overflow: hidden;
    }
    .viewport {
      position: absolute;
      inset: 0;
      overflow: hidden;
      cursor: grab;
    }
    .viewport.dragging {
      cursor: grabbing;
    }
    .scene {
      position: absolute;
      left: 50%;
      top: 50%;
      transform-origin: center center;
      will-change: transform;
    }
    #mapImg {
      display: block;
      user-select: none;
      -webkit-user-drag: none;
      border: 1px solid #333;
      image-rendering: pixelated;
      background: #6f6f6f;
    }
    #overlay {
      position: absolute;
      left: 0;
      top: 0;
      pointer-events: none;
    }
    .toolbar {
      position: absolute;
      top: 12px;
      left: 12px;
      z-index: 20;
      display: flex;
      flex-direction: column;
      gap: 8px;
      background: rgba(0,0,0,0.62);
      padding: 10px;
      border-radius: 8px;
      border: 1px solid #2d2d2d;
      min-width: 250px;
    }
    .row {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      align-items: center;
    }
    .btn {
      padding: 8px 10px;
      font-size: 13px;
      border: none;
      border-radius: 6px;
      cursor: pointer;
      font-weight: 700;
      color: white;
      background: #2f2f2f;
    }
    .btn-blue { background: #1e88e5; }
    .btn-danger { background: #e53935; }
    .btn-toggle.active { outline: 2px solid #fff; }
    .panel {
      position: absolute;
      right: 12px;
      top: 12px;
      z-index: 20;
      background: rgba(0,0,0,0.62);
      padding: 12px;
      border-radius: 8px;
      border: 1px solid #2d2d2d;
      min-width: 280px;
      max-width: 360px;
      font-size: 13px;
      line-height: 1.45;
    }
    .panel h3 {
      margin: 0 0 8px 0;
      font-size: 15px;
    }
    .ok { color: #6DFF7A; }
    .bad { color: #FF6B6B; }
    .warn { color: #FFD54F; }
    .small {
      color: #bbb;
      font-size: 12px;
    }
    .mono { font-family: Consolas, monospace; }
    label.cb {
      display: inline-flex;
      align-items: center;
      gap: 4px;
      font-size: 12px;
      color: #ddd;
    }
    input[type="file"] {
      display: none;
    }
  </style>
</head>
<body>
  <div class="wrap">
    <div class="toolbar">
      <div class="row">
        <button id="viewBtn" class="btn btn-blue btn-toggle active">VIEW</button>
        <button id="navBtn" class="btn btn-toggle">NAV</button>
      </div>

      <div class="row">
        <label class="cb"><input type="checkbox" id="showRobot" checked/>Robot</label>
        <label class="cb"><input type="checkbox" id="showPath" checked/>Path</label>
        <label class="cb"><input type="checkbox" id="showGrid"/>Grid</label>
      </div>

      <div id="viewPanel">
        <div class="row">
          <button id="saveBtn" class="btn btn-blue">SAVE MAP</button>
          <button id="loadBtn" class="btn btn-blue">LOAD MAP</button>
          <input id="mapFile" type="file" accept=".yaml,.yml,.pgm,.png,.jpg,.jpeg"/>
        </div>
        <div class="row">
          <button id="resetViewBtn" class="btn">RESET VIEW</button>
        </div>
      </div>

      <div id="navPanel" style="display:none">
        <div class="row">
          <button id="clearBtn" class="btn btn-danger">STOP & CLEAR PATH</button>
        </div>
        <div class="small">
          NAV mode: click 1 lan chon diem, click lan 2 chon huong.
        </div>
      </div>
    </div>

    <div id="statusPanel" class="panel" style="display:none">
      <h3>System status</h3>
      <div id="statusBox">Waiting...</div>
      <hr style="border-color:#333">
      <div id="goalInfo">Goal: none</div>
      <div id="cursorInfo" class="small">Coordinate: --</div>
    </div>

    <div id="viewport" class="viewport">
      <div id="scene" class="scene">
        <img id="mapImg" src="map.png" />
        <canvas id="overlay"></canvas>
      </div>
    </div>
  </div>

  <script>
    const viewport = document.getElementById("viewport");
    const scene = document.getElementById("scene");
    const mapImg = document.getElementById("mapImg");
    const overlay = document.getElementById("overlay");
    const ctx = overlay.getContext("2d");

    const viewBtn = document.getElementById("viewBtn");
    const navBtn = document.getElementById("navBtn");
    const viewPanel = document.getElementById("viewPanel");
    const navPanel = document.getElementById("navPanel");
    const statusPanel = document.getElementById("statusPanel");

    const saveBtn = document.getElementById("saveBtn");
    const loadBtn = document.getElementById("loadBtn");
    const resetViewBtn = document.getElementById("resetViewBtn");
    const clearBtn = document.getElementById("clearBtn");
    const mapFile = document.getElementById("mapFile");

    const showRobot = document.getElementById("showRobot");
    const showPath = document.getElementById("showPath");
    const showGrid = document.getElementById("showGrid");

    const statusBox = document.getElementById("statusBox");
    const goalInfo = document.getElementById("goalInfo");
    const cursorInfo = document.getElementById("cursorInfo");

    let scale = 1.0;
    let tx = 0;
    let ty = 0;
    let dragging = false;
    let dragMoved = false;
    let lastX = 0;
    let lastY = 0;

    let mapVersion = -1;
    let lastState = null;
    let uiMode = "view";

    let pendingGoal = null;
    let mouseImg = null;

    function api(path, options) {
      return fetch(path, options);
    }

    function setToggleButton(btn, active) {
      btn.classList.toggle("active", !!active);
    }

    function switchMode(mode) {
      uiMode = mode;
      const isView = mode === "view";

      viewPanel.style.display = isView ? "block" : "none";
      navPanel.style.display = isView ? "none" : "block";
      statusPanel.style.display = isView ? "none" : "block";

      setToggleButton(viewBtn, isView);
      setToggleButton(navBtn, !isView);

      pendingGoal = null;
      mouseImg = null;
      drawOverlay();
    }

    function applyTransform() {
      scene.style.transform = `translate(${tx}px, ${ty}px) scale(${scale}) translate(-50%, -50%)`;
    }

    function resetView() {
      scale = 1.0;
      tx = 0;
      ty = 0;
      applyTransform();
      drawOverlay();
    }

    function syncOverlaySize() {
      overlay.width = mapImg.clientWidth;
      overlay.height = mapImg.clientHeight;
      overlay.style.width = mapImg.clientWidth + "px";
      overlay.style.height = mapImg.clientHeight + "px";
      drawOverlay();
    }

    function fmtNum(v, d=2) {
      if (v === null || v === undefined || Number.isNaN(v)) return "--";
      return Number(v).toFixed(d);
    }

    function fmtAge(v) {
      if (v === null || v === undefined) return "--";
      return `${v.toFixed(1)} s`;
    }

    function getRenderInfo() {
      return lastState ? lastState.render_info : null;
    }

    function worldToImage(wx, wy) {
      const r = getRenderInfo();
      if (!r) return null;
      const widthM = r.width_cells * r.resolution;
      const heightM = r.height_cells * r.resolution;
      const u = (wx - r.origin_x) / widthM;
      const v = (wy - r.origin_y) / heightM;
      return {
        x: u * overlay.width,
        y: (1.0 - v) * overlay.height
      };
    }

    function imageToWorld(ix, iy) {
      const r = getRenderInfo();
      if (!r) return null;
      const u = ix / overlay.width;
      const v = 1.0 - (iy / overlay.height);
      const widthM = r.width_cells * r.resolution;
      const heightM = r.height_cells * r.resolution;
      return {
        x: r.origin_x + u * widthM,
        y: r.origin_y + v * heightM,
        u: u,
        v: v
      };
    }

    // SUA CHINH: quy doi client -> anh goc dung theo viewport center + pan/zoom
    function clientToImageLocal(clientX, clientY) {
      const vpRect = viewport.getBoundingClientRect();

      const cx = vpRect.left + vpRect.width / 2.0;
      const cy = vpRect.top + vpRect.height / 2.0;

      const imgW = mapImg.clientWidth;
      const imgH = mapImg.clientHeight;

      const x = ((clientX - cx - tx) / scale) + imgW / 2.0;
      const y = ((clientY - cy - ty) / scale) + imgH / 2.0;

      return {
        x: x,
        y: y,
        rect: {
          width: imgW,
          height: imgH
        }
      };
    }

    function drawPath(list, color, width) {
      if (!list || list.length < 2) return;
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.beginPath();
      let started = false;
      for (const p of list) {
        const q = worldToImage(p.x, p.y);
        if (!q) continue;
        if (!started) {
          ctx.moveTo(q.x, q.y);
          started = true;
        } else {
          ctx.lineTo(q.x, q.y);
        }
      }
      ctx.stroke();
    }

    function drawRobot(x, y, yaw) {
      if (!showRobot.checked) return;

      const p0 = worldToImage(x, y);
      const px = worldToImage(x + 0.28 * Math.cos(yaw), y + 0.28 * Math.sin(yaw));
      const py = worldToImage(x + 0.28 * Math.cos(yaw + Math.PI / 2.0), y + 0.28 * Math.sin(yaw + Math.PI / 2.0));

      if (!p0) return;

      ctx.strokeStyle = "cyan";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(p0.x, p0.y, 8, 0, Math.PI * 2);
      ctx.stroke();

      if (px) {
        ctx.strokeStyle = "red";
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(p0.x, p0.y);
        ctx.lineTo(px.x, px.y);
        ctx.stroke();
      }

      if (py) {
        ctx.strokeStyle = "lime";
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(p0.x, p0.y);
        ctx.lineTo(py.x, py.y);
        ctx.stroke();
      }
    }

    function drawGoal(goal) {
      if (!goal || goal.x === null || goal.y === null || goal.yaw === null) return;
      const p = worldToImage(goal.x, goal.y);
      if (!p) return;

      ctx.strokeStyle = "yellow";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(p.x, p.y, 6, 0, Math.PI * 2);
      ctx.stroke();

      const p2 = worldToImage(
        goal.x + 0.22 * Math.cos(goal.yaw),
        goal.y + 0.22 * Math.sin(goal.yaw)
      );
      if (p2) {
        ctx.beginPath();
        ctx.moveTo(p.x, p.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
      }
    }

    function drawPendingGoal() {
      if (!pendingGoal) return;

      ctx.strokeStyle = "#00ffff";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(pendingGoal.imgx, pendingGoal.imgy, 6, 0, Math.PI * 2);
      ctx.stroke();

      if (mouseImg) {
        ctx.strokeStyle = "#ffd54f";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(pendingGoal.imgx, pendingGoal.imgy);
        ctx.lineTo(mouseImg.x, mouseImg.y);
        ctx.stroke();
      }
    }

    function drawGridOverlay() {
      if (!showGrid.checked) return;
      const r = getRenderInfo();
      if (!r) return;

      const widthM = r.width_cells * r.resolution;
      const heightM = r.height_cells * r.resolution;

      const step = 0.5;

      const startX = Math.floor(r.origin_x / step) * step;
      const endX = r.origin_x + widthM;
      for (let x = startX; x <= endX + 1e-6; x += step) {
        const p1 = worldToImage(x, r.origin_y);
        const p2 = worldToImage(x, r.origin_y + heightM);
        if (!p1 || !p2) continue;
        const isMajor = Math.abs((x / 1.0) - Math.round(x / 1.0)) < 1e-6;
        ctx.strokeStyle = isMajor ? "rgba(255,255,255,0.18)" : "rgba(255,255,255,0.08)";
        ctx.lineWidth = isMajor ? 1.0 : 0.5;
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
      }

      const startY = Math.floor(r.origin_y / step) * step;
      const endY = r.origin_y + heightM;
      for (let y = startY; y <= endY + 1e-6; y += step) {
        const p1 = worldToImage(r.origin_x, y);
        const p2 = worldToImage(r.origin_x + widthM, y);
        if (!p1 || !p2) continue;
        const isMajor = Math.abs((y / 1.0) - Math.round(y / 1.0)) < 1e-6;
        ctx.strokeStyle = isMajor ? "rgba(255,255,255,0.18)" : "rgba(255,255,255,0.08)";
        ctx.lineWidth = isMajor ? 1.0 : 0.5;
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
      }
    }

    function drawOverlay() {
      ctx.clearRect(0, 0, overlay.width, overlay.height);

      if (!lastState || !lastState.render_info) {
        drawPendingGoal();
        return;
      }

      drawGridOverlay();

      if (showPath.checked) {
        drawPath(lastState.paths.a_star, "#ff2d55", 2.5);
      }

      if (lastState.pose && lastState.pose.ok) {
        drawRobot(lastState.pose.x, lastState.pose.y, lastState.pose.theta);
      }

      if (uiMode === "nav") {
        drawGoal(lastState.goal);
        drawPendingGoal();
      }
    }

    function updateStatus(state) {
      const slamOk = !!state.status.slam_ok;
      const tfOk = !!state.status.tf_ok;
      const plannerOk = !!state.status.planner_ok;
      const nav2Ok = !!state.status.nav2_server_ok;
      const nav2Active = !!state.status.nav2_goal_active;

      const slamClass = slamOk ? "ok" : "bad";
      const tfClass = tfOk ? "ok" : "bad";
      const plannerClass = plannerOk ? "ok" : "warn";
      const nav2Class = nav2Ok ? "ok" : "bad";
      const nav2ActiveClass = nav2Active ? "warn" : "small";


      const pose = state.pose || {};
      const goal = state.goal || {};

      statusBox.innerHTML = `
        <div>SLAM: <span class="${slamClass}">${slamOk ? "OK" : "NO MAP"}</span></div>
        <div>TF: <span class="${tfClass}">${tfOk ? "OK" : "TIMEOUT"}</span></div>
        <div>Nav2 server: <span class="${nav2Class}">${nav2Ok ? "READY" : "DOWN"}</span></div>
        <div>Planner: <span class="${plannerClass}">${plannerOk ? "OK" : "WAIT/FAIL"}</span></div>
        <div>Nav2 goal: <span class="${nav2ActiveClass}">${nav2Active ? "ACTIVE" : (state.status.nav2_result || "idle")}</span></div>
        <div>Robot: <span class="mono">x=${fmtNum(pose.x)}, y=${fmtNum(pose.y)}, yaw=${fmtNum((pose.theta || 0) * 180.0 / Math.PI, 1)} deg</span></div>
        <div>Map age: ${fmtAge(state.status.map_age_sec)}</div>
        <div>Pose age: ${fmtAge(state.status.pose_age_sec)}</div>
      `;


      if (goal.x === null || goal.y === null || goal.yaw === null) {
        goalInfo.innerText = "Goal: none";
      } else {
        goalInfo.innerText =
          `Goal: x=${goal.x.toFixed(2)}, y=${goal.y.toFixed(2)}, yaw=${(goal.yaw * 180.0 / Math.PI).toFixed(1)} deg`;
      }
    }

    async function fetchState() {
      try {
        const res = await api("/state");
        const state = await res.json();
        lastState = state;

        if (state.map_version !== mapVersion) {
          mapVersion = state.map_version;
          mapImg.src = "/map.png?v=" + mapVersion + "&t=" + Date.now();
        }

        if (uiMode === "nav") {
          updateStatus(state);
        }

        drawOverlay();
      } catch (_) {
      }
    }

    async function sendGoal(u, v, yaw) {
      const url = `/set_goal_pose?u=${encodeURIComponent(u)}&v=${encodeURIComponent(v)}&yaw=${encodeURIComponent(yaw)}`;
      const res = await api(url);
      if (!res.ok) throw new Error("set_goal_pose failed");
    }

    mapImg.addEventListener("load", syncOverlaySize);
    window.addEventListener("resize", syncOverlaySize);

    viewport.addEventListener("wheel", (e) => {
      e.preventDefault();
      const k = e.deltaY < 0 ? 1.1 : 0.9;
      scale = Math.max(0.25, Math.min(10.0, scale * k));
      applyTransform();
    }, { passive: false });

    viewport.addEventListener("mousedown", (e) => {
      if (e.button !== 0) return;
      dragging = true;
      dragMoved = false;
      lastX = e.clientX;
      lastY = e.clientY;
      viewport.classList.add("dragging");
    });

    window.addEventListener("mouseup", () => {
      dragging = false;
      viewport.classList.remove("dragging");
    });

    window.addEventListener("mousemove", (e) => {
      if (dragging) {
        const dx = e.clientX - lastX;
        const dy = e.clientY - lastY;
        if (Math.abs(dx) > 2 || Math.abs(dy) > 2) {
          dragMoved = true;
        }
        tx += dx;
        ty += dy;
        lastX = e.clientX;
        lastY = e.clientY;
        applyTransform();
      }

      const p = clientToImageLocal(e.clientX, e.clientY);
      if (uiMode === "nav" && p.x >= 0 && p.y >= 0 && p.x <= p.rect.width && p.y <= p.rect.height && lastState && lastState.render_info) {
        const w = imageToWorld(p.x, p.y);
        if (w) {
          cursorInfo.innerText = `Coordinate: x=${w.x.toFixed(2)}, y=${w.y.toFixed(2)}`;
        }
      } else if (uiMode === "nav") {
        cursorInfo.innerText = "Coordinate: --";
      }

      if (pendingGoal) {
        mouseImg = {x: p.x, y: p.y};
        drawOverlay();
      }
    });

    mapImg.addEventListener("click", async (e) => {
      if (uiMode !== "nav") return;
      if (dragMoved) return;
      if (!lastState || !lastState.render_info) return;

      const p = clientToImageLocal(e.clientX, e.clientY);
      if (p.x < 0 || p.y < 0 || p.x > p.rect.width || p.y > p.rect.height) return;

      const w = imageToWorld(p.x, p.y);
      if (!w) return;

      try {
        if (!pendingGoal) {
          pendingGoal = {
            imgx: p.x,
            imgy: p.y,
            u: w.u,
            v: w.v,
          };
          mouseImg = {x: p.x, y: p.y};
          drawOverlay();
          return;
        }

        const dx = p.x - pendingGoal.imgx;
        const dy = p.y - pendingGoal.imgy;
        const yaw = Math.atan2(-dy, dx);

        await sendGoal(pendingGoal.u, pendingGoal.v, yaw);

        pendingGoal = null;
        mouseImg = null;
        drawOverlay();
      } catch (err) {
        alert("Set goal failed: " + err);
      }
    });

    viewBtn.addEventListener("click", () => switchMode("view"));
    navBtn.addEventListener("click", () => switchMode("nav"));

    resetViewBtn.addEventListener("click", resetView);

    clearBtn.addEventListener("click", async () => {
      try { await api("/clear_path"); } catch (_) {}
    });

    saveBtn.addEventListener("click", async () => {
      const name = prompt("Map name (letters/numbers only):", "room1");
      if (!name) return;
      const safe = name.trim().replace(/[^a-zA-Z0-9_\\-]/g, "_");
      if (!safe) return;

      try {
        const res = await api(`/save_map?name=${encodeURIComponent(safe)}`);
        if (!res.ok) throw new Error("save_map failed");
        setTimeout(() => {
          window.open(`/maps/${encodeURIComponent(safe)}.yaml`, "_blank");
        }, 1000);
      } catch (e) {
        alert("Save map failed: " + e);
      }
    });

    loadBtn.addEventListener("click", () => mapFile.click());

    mapFile.addEventListener("change", async (e) => {
      const file = e.target.files[0];
      if (!file) return;
      const formData = new FormData();
      formData.append("file", file);
      try {
        const res = await api("/upload_map", { method: "POST", body: formData });
        if (!res.ok) throw new Error("Upload failed");
        alert("Uploaded: " + file.name);
      } catch (err) {
        alert("Upload error: " + err);
      } finally {
        mapFile.value = "";
      }
    });

    showRobot.addEventListener("change", drawOverlay);
    showPath.addEventListener("change", drawOverlay);
    showGrid.addEventListener("change", drawOverlay);

    applyTransform();
    switchMode("view");
    fetchState();
    setInterval(fetchState, 300);
  </script>
</body>
</html>
"""


class LiveMapWeb(Node):
    def __init__(self):
        super().__init__("slam_live_map_viewer")

        self.map_topic = os.environ.get("MAP_TOPIC", "/map")
        self.base_frame = os.environ.get("BASE_FRAME", "base_link")
        self.nav_path_topic = os.environ.get("NAV_PATH_TOPIC", "/plan")

        self.map_msg = None
        self.map_frame = "map"
        self.path_xy = None

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.current_goal_handle = None
        self.current_goal_future = None
        self.current_result_future = None


        self.map_last_stamp_wall = None
        self.pose_last_stamp_wall = None

        self.map_dirty = False
        self.last_map_fingerprint = None

        os.makedirs(BASE_DIR, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Path,
            self.nav_path_topic,
            self.cb_nav_path,
            10,
        )


        self.path_pub = self.create_publisher(Path, "/a_star_path", 1)

        self.get_logger().info(
            f"[slam_live_map] HTTP: http://<IP_RPI>:8080/ | map={self.map_topic} | base_frame={self.base_frame}"
        )

        self.fast_timer = self.create_timer(0.15, self.fast_tick)
        self.slow_timer = self.create_timer(1.0, self.slow_tick)

    def cb_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.map_frame = msg.header.frame_id or "map"
        self.map_last_stamp_wall = now_sec()

        h = msg.info.height
        w = msg.info.width
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        occ_mask = data > 50
        free_mask = data == 0

        grid = np.zeros((h, w), dtype=np.uint8)
        grid[occ_mask] = 1
        self.grid = grid

        try:
            fp = (
                int(w),
                int(h),
                float(msg.info.resolution),
                float(msg.info.origin.position.x),
                float(msg.info.origin.position.y),
                int(np.count_nonzero(occ_mask)),
                int(np.count_nonzero(free_mask)),
            )
        except Exception:
            fp = (int(w), int(h), now_sec())

        if fp != self.last_map_fingerprint:
            self.last_map_fingerprint = fp
            self.map_dirty = True

        def _upd(state):
            state["map_info"] = {
                "width": int(w),
                "height": int(h),
                "resolution": float(msg.info.resolution),
                "origin_x": float(msg.info.origin.position.x),
                "origin_y": float(msg.info.origin.position.y),
                "frame_id": self.map_frame,
            }
            state["status"]["slam_ok"] = True
            state["status"]["last_update"] = now_sec()

        update_shared_state(_upd)

    def process_goal_request_if_any(self):
        req = pop_goal_request()
        if req is None or self.map_msg is None or self.grid is None:
            return

        u = req["u"]
        v = req["v"]
        goal_yaw = req["yaw"]

        render_info = get_state_snapshot().get("render_info")
        if render_info is None:
            self.get_logger().warn("render_info is None")
            return

        res = render_info["resolution"]
        ox = render_info["origin_x"]
        oy = render_info["origin_y"]
        w = render_info["width_cells"]
        h = render_info["height_cells"]

        c_grid = int(u * (w - 1))
        r_grid = int(v * (h - 1))

        c_grid = max(0, min(c_grid, w - 1))
        r_grid = max(0, min(r_grid, h - 1))

        gx = ox + (c_grid + 0.5) * res
        gy = oy + (r_grid + 0.5) * res

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"Khong lay duoc TF {self.map_frame}->{self.base_frame}: {e}")
            return

        def _upd_goal(state):
            state["goal"]["x"] = float(gx)
            state["goal"]["y"] = float(gy)
            state["goal"]["yaw"] = float(goal_yaw)
            state["goal"]["stamp"] = now_sec()
            state["status"]["planner_ok"] = False
            state["status"]["planner_msg"] = "planning..."
            state["status"]["last_update"] = now_sec()

        update_shared_state(_upd_goal)

        self.get_logger().info(
            f"Goal world=({gx:.2f},{gy:.2f}), yaw={math.degrees(goal_yaw):.1f} deg | robot=({rx:.2f},{ry:.2f})"
        )

        try:
            self.path_xy = plan_path(
                self.grid,
                self.map_msg.info,
                (rx, ry),
                (gx, gy),
                logger=self.get_logger(),
                safe_clearance_m=self.safe_clearance_m,
            )
        except Exception as e:
            self.path_xy = None
            self.get_logger().error(f"Planner error: {e}")

        if self.path_xy and len(self.path_xy) >= 2:
            self.publish_path_and_goal(self.path_xy, self.map_frame, goal_yaw)

            pts = [{"x": float(x), "y": float(y)} for (x, y) in self.path_xy]

            def _upd_ok(state):
                state["paths"]["a_star"] = pts
                state["status"]["planner_ok"] = True
                state["status"]["planner_msg"] = f"path ok ({len(pts)} pts)"
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd_ok)
        else:
            self.get_logger().warn("Planner khong tra ve duong di hop le.")

            def _upd_fail(state):
                state["paths"]["a_star"] = []
                state["status"]["planner_ok"] = False
                state["status"]["planner_msg"] = "planner failed"
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd_fail)

    def process_clear_request_if_any(self):
        if not pop_clear_request():
            return

        self.path_xy = None

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame
        self.path_pub.publish(path_msg)

        def _upd(state):
            state["goal"]["x"] = None
            state["goal"]["y"] = None
            state["goal"]["yaw"] = None
            state["goal"]["stamp"] = now_sec()
            state["paths"]["a_star"] = []
            state["status"]["planner_ok"] = False
            state["status"]["planner_msg"] = "cleared"
            state["status"]["last_update"] = now_sec()

        update_shared_state(_upd)
        self.get_logger().info("Clear path request: published empty /a_star_path.")

    def process_save_request_if_any(self):
        name = pop_save_request()
        if not name:
            return
        if self.map_msg is None:
            self.get_logger().warn("SAVE_MAP: map_msg is None, cannot save.")
            return

        try:
            self.save_map_files(name, self.map_msg)
            self.get_logger().info(f"SAVE_MAP: saved map '{name}' into {MAP_SAVE_DIR}")
        except Exception as e:
            self.get_logger().error(f"SAVE_MAP failed: {e}")

    def save_map_files(self, base_name: str, msg: OccupancyGrid):
        h = msg.info.height
        w = msg.info.width
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        img = np.full((h, w), 205, dtype=np.uint8)
        img[data == 0] = 254
        img[data > 50] = 0

        pgm_path = os.path.join(MAP_SAVE_DIR, base_name + ".pgm")
        yaml_path = os.path.join(MAP_SAVE_DIR, base_name + ".yaml")

        with open(pgm_path, "wb") as f:
            header = f"P5\n{w} {h}\n255\n"
            f.write(header.encode("ascii"))
            f.write(np.flipud(img).tobytes())

        origin = msg.info.origin
        yaml_content = (
            f"image: {base_name}.pgm\n"
            f"resolution: {msg.info.resolution}\n"
            f"origin: [{origin.position.x}, {origin.position.y}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )

        with open(yaml_path, "w") as f:
            f.write(yaml_content)

    def publish_path_and_goal(self, path_xy, frame_id: str, goal_yaw: float):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id

        for (x, y) in path_xy:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = path_msg.header.stamp
        goal_pose.pose.position.x = path_xy[-1][0]
        goal_pose.pose.position.y = path_xy[-1][1]
        goal_pose.pose.position.z = 0.0
        qz, qw = yaw_to_quat(goal_yaw)
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw
        self.goal_pub.publish(goal_pose)

        self.get_logger().info(
            f"Published path with {len(path_msg.poses)} poses to /a_star_path"
        )

    def render_map_png_if_needed(self):
        if not self.map_dirty or self.map_msg is None:
            return

        msg = self.map_msg
        h = int(msg.info.height)
        w = int(msg.info.width)
        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)

        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        side = max(w, h)
        extra = max(20, int(0.15 * side))
        side2 = side + 2 * extra

        pad_left = (side2 - w) // 2
        pad_bottom = (side2 - h) // 2

        rgb = np.zeros((side2, side2, 3), dtype=np.float32)
        rgb[:] = [0.45, 0.45, 0.45]

        map_rgb = np.zeros((h, w, 3), dtype=np.float32)
        map_rgb[:] = [0.5, 0.5, 0.5]
        map_rgb[data == 0] = [1.0, 1.0, 1.0]
        map_rgb[data > 50] = [0.0, 0.0, 0.0]

        rgb[pad_bottom:pad_bottom + h, pad_left:pad_left + w, :] = map_rgb
        rgb = np.flipud(rgb)

        tmp_path = os.path.join(BASE_DIR, "map_tmp.png")
        plt.imsave(tmp_path, rgb)
        os.replace(tmp_path, MAP_PNG_PATH)

        render_origin_x = ox - pad_left * res
        render_origin_y = oy - pad_bottom * res

        self.map_dirty = False

        def _upd(state):
            state["map_version"] += 1
            state["render_info"] = {
                "width_cells": int(side2),
                "height_cells": int(side2),
                "resolution": float(res),
                "origin_x": float(render_origin_x),
                "origin_y": float(render_origin_y),
                "pad_left_cells": int(pad_left),
                "pad_bottom_cells": int(pad_bottom),
            }
            state["status"]["last_update"] = now_sec()

        update_shared_state(_upd)

    def update_pose_and_status(self):
        tnow = now_sec()
        nav2_ok = self.nav_to_pose_client.server_is_ready()
        tf_ok = False
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.08),
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = quat_to_yaw(tf.transform.rotation)

            self.pose_last_stamp_wall = tnow
            tf_ok = True

            def _upd(state):
                state["pose"]["x"] = float(x)
                state["pose"]["y"] = float(y)
                state["pose"]["theta"] = float(yaw)
                state["pose"]["stamp"] = tnow
                state["pose"]["ok"] = True
                state["status"]["tf_ok"] = True
                state["status"]["last_update"] = tnow

            update_shared_state(_upd)

        except Exception:
            def _upd(state):
                state["pose"]["ok"] = False
                state["status"]["tf_ok"] = False
                state["status"]["last_update"] = tnow

            update_shared_state(_upd)

        def age_of(ts):
            if ts is None:
                return None
            return max(0.0, tnow - ts)

        def _upd_age(state):
            state["status"]["slam_ok"] = self.map_msg is not None
            state["status"]["tf_ok"] = tf_ok
            state["status"]["nav2_server_ok"] = bool(nav2_ok)
            state["status"]["map_age_sec"] = age_of(self.map_last_stamp_wall)
            state["status"]["pose_age_sec"] = age_of(self.pose_last_stamp_wall)
            state["status"]["last_update"] = tnow

        update_shared_state(_upd_age)

    def fast_tick(self):
        self.process_clear_request_if_any()
        self.process_save_request_if_any()
        self.process_goal_request_if_any()
        self.update_pose_and_status()

    def slow_tick(self):
        self.render_map_png_if_needed()


def start_web_server():
    httpd = HTTPServer(("0.0.0.0", 8080), ImageServer)
    httpd.serve_forever()


def main():
    threading.Thread(target=start_web_server, daemon=True).start()

    rclpy.init()
    node = LiveMapWeb()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

