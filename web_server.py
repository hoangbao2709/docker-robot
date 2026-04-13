#!/usr/bin/env python3
# coding=utf-8

import cgi
import json
import os
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import parse_qs, urlparse
from points_store import load_points, upsert_point, get_point, delete_point
from cartographer_manager import get_slam_status
from config import MAP_PNG_PATH, MAP_SAVE_DIR
from shared_state import (
    get_state_snapshot,
    request_clear_path,
    set_goal_request,
    set_initial_pose_request,
    set_save_request,
)
from templates import build_index_html


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

    def _read_json_body(self):
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length > 0 else b"{}"
        return json.loads(raw.decode("utf-8"))
    
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
            self._send_html(build_index_html())
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

        if path == "/slam_status":
            self._send_json(200, get_slam_status())
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

