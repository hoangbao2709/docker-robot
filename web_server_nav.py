#!/usr/bin/env python3
# coding=utf-8

import cgi
import json
import os
import zipfile
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlparse
from points_store import load_points, upsert_point, get_point, delete_point
# from cartographer_manager import get_slam_status
from config import BASE_DIR, MAP_PNG_PATH, MAP_SAVE_DIR
from metrics_store import (
    get_distance_snapshot,
    get_metrics_snapshot,
    reset_distance,
    reset_metrics,
)
from shared_state import (
    clear_map_override,
    get_map_override,
    get_state_light_snapshot,
    get_state_snapshot,
    request_clear_path,
    set_map_override,
    set_goal_request,
    set_initial_pose_request,
    set_save_request,
)
from templates import build_index_html

LOADED_MAP_IMAGE_PATH = os.path.join(BASE_DIR, "_loaded_map_preview.png")


def _get_effective_state_snapshot():
    snapshot = get_state_snapshot()
    override = get_map_override()
    if override is None:
        return snapshot

    snapshot["map_version"] = override["map_version"]
    snapshot["map_info"] = override["map_info"]
    snapshot["render_info"] = override["render_info"]
    snapshot["status"]["last_update"] = snapshot["status"].get("last_update", 0.0)
    snapshot["status"]["planner_msg"] = "loaded saved map bundle"
    return snapshot


def _get_effective_state_light_snapshot():
    snapshot = get_state_light_snapshot()
    override = get_map_override()
    if override is None:
        return snapshot

    snapshot["map_version"] = override["map_version"]
    snapshot["map_info"] = override["map_info"]
    snapshot["render_info"] = override["render_info"]
    snapshot["status"]["last_update"] = snapshot["status"].get("last_update", 0.0)
    snapshot["status"]["planner_msg"] = "loaded saved map bundle"
    return snapshot


def _get_effective_map_png_path():
    override = get_map_override()
    if override and os.path.isfile(override["image_path"]):
        return override["image_path"]
    return MAP_PNG_PATH


def _activate_map_bundle(bundle_path: str):
    with zipfile.ZipFile(bundle_path, "r") as zf:
        if "metadata.json" not in zf.namelist():
            raise ValueError("bundle missing metadata.json")
        if "preview.png" not in zf.namelist():
            raise ValueError("bundle missing preview.png")

        metadata = json.loads(zf.read("metadata.json").decode("utf-8"))
        map_info = metadata.get("map_info")
        render_info = metadata.get("render_info")

        if not isinstance(map_info, dict) or not isinstance(render_info, dict):
            raise ValueError("bundle metadata invalid")

        with open(LOADED_MAP_IMAGE_PATH, "wb") as f:
            f.write(zf.read("preview.png"))

    set_map_override(
        LOADED_MAP_IMAGE_PATH,
        map_info=map_info,
        render_info=render_info,
    )

    return {
        "map_info": map_info,
        "render_info": render_info,
        "image_path": LOADED_MAP_IMAGE_PATH,
    }


class ImageServer(BaseHTTPRequestHandler):
    server_version = "LightMapHTTP/1.0"

    def _send_bytes(self, status_code, content_type, data: bytes, extra_headers=None, send_body=True):
        self.send_response(status_code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, HEAD, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        if extra_headers:
            for k, v in extra_headers.items():
                self.send_header(k, v)
        self.end_headers()
        if send_body:
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

    def _read_json_body(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length > 0 else b"{}"
        return json.loads(raw.decode("utf-8"))

    def _send_saved_map_file(self, rel_path, send_body=True):
        fname = os.path.basename(rel_path)
        fpath = os.path.join(MAP_SAVE_DIR, fname)
        if not os.path.isfile(fpath):
            self.send_error(404, "map file not found")
            return

        if fname.endswith(".yaml") or fname.endswith(".yml"):
            ctype = "text/yaml"
        elif fname.endswith(".pgm"):
            ctype = "image/x-portable-graymap"
        elif fname.endswith(".pbstream"):
            ctype = "application/x-cartographer-pbstream"
        else:
            ctype = "application/octet-stream"

        with open(fpath, "rb") as f:
            data = f.read()

        self._send_bytes(
            200,
            ctype,
            data,
            extra_headers={"Content-Disposition": f'attachment; filename="{fname}"'},
            send_body=send_body,
        )
    
    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, HEAD, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_HEAD(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path.startswith("/maps/"):
            self._send_saved_map_file(path[len("/maps/"):], send_body=False)
            return
        self.send_error(404)

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/" or path == "/index.html":
            self._send_html(build_index_html())
            return

        if path == "/state":
            self._send_json(200, _get_effective_state_snapshot())
            return

        if path == "/state_light":
            self._send_json(200, _get_effective_state_light_snapshot())
            return

        if path == "/map.png":
            try:
                with open(_get_effective_map_png_path(), "rb") as f:
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

        if path == "/slam_status":
            self._send_json(200,  {        "running": False,        "mode": "navigation",        "message": "SLAM disabled"})
            return

        if path == "/metrics":
            self._send_json(200, get_metrics_snapshot())
            return

        if path == "/distance":
            self._send_json(200, get_distance_snapshot())
            return

        if path.startswith("/set_goal_pose"):
            try:
                qs = parse_qs(parsed.query)
                x = float(qs.get("x", [None])[0])
                y = float(qs.get("y", [None])[0])
                yaw = float(qs.get("yaw", [0.0])[0])
                u_raw = qs.get("u", [None])[0]
                v_raw = qs.get("v", [None])[0]
                u = None if u_raw is None else float(u_raw)
                v = None if v_raw is None else float(v_raw)

                set_goal_request(x, y, yaw, u=u, v=v)
                self._send_text(200, "OK")
            except Exception as e:
                self.send_error(400, f"bad params: {e}")
            return
        if path.startswith("/set_initial_pose"):
            try:
                qs = parse_qs(parsed.query)
                x = float(qs.get("x", [None])[0])
                y = float(qs.get("y", [None])[0])
                yaw = float(qs.get("yaw", [0.0])[0])
                u_raw = qs.get("u", [None])[0]
                v_raw = qs.get("v", [None])[0]
                u = None if u_raw is None else float(u_raw)
                v = None if v_raw is None else float(v_raw)

                set_initial_pose_request(x, y, yaw, u=u, v=v)
                self._send_text(200, "OK")
            except Exception as e:
                self.send_error(400, f"bad params: {e}")
            return
        if path.startswith("/clear_path"):
            request_clear_path()
            self._send_text(200, "CLEARED")
            return
        if path == "/points":
            self._send_json(200, load_points())
            return

        if path.startswith("/points/"):
            name = os.path.basename(path)
            point = get_point(name)
            if point is None:
                self.send_error(404, "point not found")
                return
            self._send_json(200, {"name": name, **point})
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

        if path == "/use_live_map":
            clear_map_override()
            self._send_text(200, "LIVE_MAP_ENABLED")
            return

        if path.startswith("/maps/"):
            self._send_saved_map_file(path[len("/maps/"):])
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

                if ext not in (".yaml", ".yml", ".pgm", ".png", ".jpg", ".jpeg", ".zip", ".pbstream"):
                    self.send_error(400, "Unsupported file type")
                    return

                fpath = os.path.join(MAP_SAVE_DIR, safe)
                with open(fpath, "wb") as f:
                    f.write(field_item.file.read())

                if ext == ".zip":
                    bundle_info = _activate_map_bundle(fpath)
                    self._send_json(200, {
                        "success": True,
                        "message": f"LOADED {safe}",
                        "bundle": safe,
                        "map_info": bundle_info["map_info"],
                    })
                    return

                self._send_text(200, f"UPLOADED {safe}")
            except Exception as e:
                self.send_error(500, f"Upload error: {e}")
            return
        if path == "/points":
            try:
                body = self._read_json_body()
                name = str(body.get("name", "")).strip()
                x = float(body.get("x"))
                y = float(body.get("y"))
                yaw = float(body.get("yaw", 0.0))

                if not name:
                    raise ValueError("name is required")

                safe_name = "".join(ch for ch in name if ch.isalnum() or ch in "_-")
                if not safe_name:
                    raise ValueError("invalid point name")

                point = upsert_point(safe_name, x, y, yaw)
                self._send_json(200, {
                    "success": True,
                    "name": safe_name,
                    "point": point,
                })
            except Exception as e:
                self.send_error(400, f"bad request: {e}")
            return

        if path == "/metrics/reset":
            reset_metrics()
            self._send_json(200, {
                "success": True,
                "message": "metrics reset",
            })
            return

        if path == "/distance/reset":
            reset_distance()
            self._send_json(200, {
                "success": True,
                "message": "distance reset",
                "distance": get_distance_snapshot(),
            })
            return

        if path == "/go_to_point":
            try:
                body = self._read_json_body()
                name = str(body.get("name", "")).strip()

                if not name:
                    raise ValueError("name is required")

                point = get_point(name)
                if point is None:
                    self.send_error(404, "point not found")
                    return

                set_goal_request(
                    point["x"],
                    point["y"],
                    point.get("yaw", 0.0),
                )

                self._send_json(200, {
                    "success": True,
                    "message": f"goal '{name}' sent",
                    "goal": {
                        "name": name,
                        "x": point["x"],
                        "y": point["y"],
                        "yaw": point.get("yaw", 0.0),
                    }
                })
            except Exception as e:
                self.send_error(400, f"bad request: {e}")
            return
        if path == "/delete_point":
            try:
                body = self._read_json_body()
                name = str(body.get("name", "")).strip()
                if not name:
                    raise ValueError("name is required")

                deleted = delete_point(name)
                if deleted is None:
                    self.send_error(404, "point not found")
                    return

                self._send_json(200, {
                    "success": True,
                    "deleted": name
                })
            except Exception as e:
                self.send_error(400, f"bad request: {e}")
            return

        if path == "/upload_map":
            ...
            return

        self.send_error(404)

    def log_message(self, fmt, *args):
        return


def start_web_server(host="0.0.0.0", port=8080):
    httpd = HTTPServer((host, port), ImageServer)
    httpd.serve_forever()

