#!/usr/bin/env python3
# coding=utf-8

import json
import math
import os
import shutil
import zipfile
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from config import BASE_DIR, MAP_PNG_PATH, MAP_SAVE_DIR
from path_planner import plan_path
from shared_state import (
    get_state_snapshot,
    pop_clear_request,
    pop_goal_request,
    pop_initial_pose_request,
    pop_save_request,
    update_shared_state,
)
from utils import now_sec, quat_to_yaw, yaw_to_quat


class LiveMapWeb(Node):
    def __init__(self):
        super().__init__("slam_live_map_viewer")

        self.map_topic = os.environ.get("MAP_TOPIC", "/map")
        self.scan_topic = os.environ.get("SCAN_TOPIC", "/scan")
        self.base_frame = os.environ.get("BASE_FRAME", "base_link")
        self.safe_clearance_m = float(os.environ.get("SAFE_CLEARANCE_M", "0.12"))
        self.scan_stride = int(os.environ.get("SCAN_STRIDE", "2"))
        self.scan_max_points = int(os.environ.get("SCAN_MAX_POINTS", "720"))

        self.map_msg = None
        self.map_frame = "map"
        self.grid = None
        self.path_xy = None

        self.map_last_stamp_wall = None
        self.pose_last_stamp_wall = None
        self.scan_last_stamp_wall = None

        self.map_dirty = False
        self.last_map_fingerprint = None

        os.makedirs(BASE_DIR, exist_ok=True)
        os.makedirs(MAP_SAVE_DIR, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self._nav_goal_future = None
        self._nav_result_future = None

        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        qos_scan = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.cb_map,
            qos_map,
        )

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.cb_scan,
            qos_scan,
        )

        self.path_pub = self.create_publisher(Path, "/a_star_path", 1)

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )

        self.get_logger().info(
            f"[slam_live_map] HTTP: http://<IP_RPI>:8080/ | map={self.map_topic} | scan={self.scan_topic} | base_frame={self.base_frame}"
        )

        self.fast_timer = self.create_timer(0.15, self.fast_tick)
        self.slow_timer = self.create_timer(1.0, self.slow_tick)

    class _SilentPlannerLogger:
        def info(self, *_args, **_kwargs):
            return

        def warn(self, *_args, **_kwargs):
            return

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

    def cb_scan(self, msg: LaserScan):
        self.scan_last_stamp_wall = now_sec()

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            def _upd_fail(state):
                state["scan"]["ok"] = False
                state["scan"]["stamp"] = now_sec()
            update_shared_state(_upd_fail)
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        yaw = quat_to_yaw(tf.transform.rotation)

        cy = math.cos(yaw)
        sy = math.sin(yaw)

        points = []
        angle = msg.angle_min
        stride = max(1, self.scan_stride)

        for i, rng in enumerate(msg.ranges):
            if i % stride != 0:
                angle += msg.angle_increment
                continue

            if not math.isfinite(rng):
                angle += msg.angle_increment
                continue
            if rng < msg.range_min or rng > msg.range_max:
                angle += msg.angle_increment
                continue

            lx = rng * math.cos(angle)
            ly = rng * math.sin(angle)

            wx = tx + cy * lx - sy * ly
            wy = ty + sy * lx + cy * ly

            points.append({"x": float(wx), "y": float(wy)})

            if len(points) >= self.scan_max_points:
                break

            angle += msg.angle_increment

        stamp_sec = (
            float(msg.header.stamp.sec) +
            float(msg.header.stamp.nanosec) * 1e-9
        )

        def _upd(state):
            state["scan"]["points"] = points
            state["scan"]["stamp"] = stamp_sec
            state["scan"]["frame_id"] = msg.header.frame_id
            state["scan"]["ok"] = True
            state["status"]["last_update"] = now_sec()

        update_shared_state(_upd)

    def send_nav2_goal(self, gx: float, gy: float, goal_yaw: float):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("NavigateToPose action server not available")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = gx
        goal_msg.pose.pose.position.y = gy
        goal_msg.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quat(goal_yaw)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Sending Nav2 goal: x={gx:.2f}, y={gy:.2f}, yaw={math.degrees(goal_yaw):.1f} deg"
        )

        self._nav_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self._nav_goal_future.add_done_callback(self._nav_goal_response_callback)
        return True

    def _nav_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Nav2 goal send failed: {e}")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal was rejected")

            def _upd(state):
                state["status"]["planner_ok"] = False
                state["status"]["planner_msg"] = "nav2 goal rejected"
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd)
            return

        self.get_logger().info("Nav2 goal accepted")
        self._nav_result_future = goal_handle.get_result_async()
        self._nav_result_future.add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Nav2 goal finished: {result}")

            def _upd(state):
                state["status"]["planner_msg"] = "nav2 goal finished"
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd)

        except Exception as e:
            self.get_logger().error(f"Nav2 goal result failed: {e}")

            def _upd(state):
                state["status"]["planner_ok"] = False
                state["status"]["planner_msg"] = f"nav2 result error: {e}"
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd)

    def publish_initial_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quat(yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        for i in range(36):
            msg.pose.covariance[i] = 0.0

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.10

        self.initialpose_pub.publish(msg)

        self.get_logger().info(
            f"Published /initialpose: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg"
        )

    def process_initial_pose_request_if_any(self):
        req = pop_initial_pose_request()
        if req is None or self.map_msg is None or self.grid is None:
            return

        init_yaw = req["yaw"]
        x = float(req["x"])
        y = float(req["y"])

        try:
            self.publish_initial_pose(x, y, init_yaw)

            def _upd(state):
                state["pose"]["x"] = float(x)
                state["pose"]["y"] = float(y)
                state["pose"]["theta"] = float(init_yaw)
                state["pose"]["stamp"] = now_sec()
                state["pose"]["ok"] = True
                state["status"]["tf_ok"] = True
                state["status"]["planner_msg"] = "initial pose sent to AMCL"
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd)

        except Exception as e:
            self.get_logger().error(f"Publish initial pose failed: {e}")

            def _upd_fail(state):
                state["status"]["planner_msg"] = f"initial pose failed: {e}"
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd_fail)

    def process_goal_request_if_any(self):
        req = pop_goal_request()
        if req is None or self.map_msg is None or self.grid is None:
            return

        goal_yaw = req["yaw"]
        requested_gx = float(req["x"])
        requested_gy = float(req["y"])

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

        try:
            self.path_xy = plan_path(
                self.grid,
                self.map_msg.info,
                (rx, ry),
                (requested_gx, requested_gy),
                logger=self.get_logger(),
                safe_clearance_m=self.safe_clearance_m,
            )
        except Exception as e:
            self.path_xy = None
            self.get_logger().error(f"Planner error: {e}")

        def _upd_goal_requested(state):
            state["goal"]["x"] = float(requested_gx)
            state["goal"]["y"] = float(requested_gy)
            state["goal"]["yaw"] = float(goal_yaw)
            state["goal"]["stamp"] = now_sec()
            state["status"]["planner_ok"] = False
            state["status"]["planner_msg"] = "planning..."
            state["status"]["last_update"] = now_sec()

        update_shared_state(_upd_goal_requested)

        if self.path_xy and len(self.path_xy) >= 2:
            planned_gx, planned_gy = self.path_xy[-1]

            self.get_logger().info(
                f"Goal requested=({requested_gx:.2f},{requested_gy:.2f}), "
                f"planner_end=({planned_gx:.2f},{planned_gy:.2f}), "
                f"yaw={math.degrees(goal_yaw):.1f} deg | robot=({rx:.2f},{ry:.2f})"
            )

            self.send_nav2_goal(requested_gx, requested_gy, goal_yaw)
            self.publish_path(self.path_xy, self.map_frame)

            pts = self.path_xy_to_display_points(
                self.path_xy,
                goal_xy=(requested_gx, requested_gy),
            )

            def _upd_ok(state):
                state["paths"]["a_star"] = pts
                state["status"]["planner_ok"] = True
                state["status"]["planner_msg"] = f"path ok ({len(pts)} pts), nav2 goal sent"
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd_ok)
        else:
            self.get_logger().warn("Planner khong tra ve duong di hop le.")
            self.send_nav2_goal(requested_gx, requested_gy, goal_yaw)

            def _upd_fail(state):
                state["paths"]["a_star"] = []
                state["status"]["planner_ok"] = False
                state["status"]["planner_msg"] = "planner failed, nav2 goal sent"
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

    def refresh_active_path(self):
        if self.map_msg is None or self.grid is None:
            return

        snapshot = get_state_snapshot()
        goal = snapshot.get("goal") or {}
        gx = goal.get("x")
        gy = goal.get("y")

        if gx is None or gy is None:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.08),
            )
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
        except Exception:
            return

        try:
            path_xy = plan_path(
                self.grid,
                self.map_msg.info,
                (rx, ry),
                (float(gx), float(gy)),
                logger=self._SilentPlannerLogger(),
                safe_clearance_m=self.safe_clearance_m,
            )
        except Exception as e:
            self.get_logger().warn(f"refresh_active_path failed: {e}")
            return

        if not path_xy or len(path_xy) < 2:
            return

        self.path_xy = path_xy
        self.publish_path(self.path_xy, self.map_frame)

        pts = self.path_xy_to_display_points(
            self.path_xy,
            goal_xy=(float(gx), float(gy)),
        )

        def _upd(state):
            state["paths"]["a_star"] = pts
            state["status"]["planner_ok"] = True
            state["status"]["planner_msg"] = f"path refresh ok ({len(pts)} pts)"
            state["status"]["last_update"] = now_sec()

        update_shared_state(_upd)

    def process_save_request_if_any(self):
        name = pop_save_request()
        if not name:
            return
        if self.map_msg is None:
            self.get_logger().warn("SAVE_MAP: map_msg is None, cannot save.")
            return

        try:
            self.render_map_png_if_needed()
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

        preview_png_path = os.path.join(MAP_SAVE_DIR, base_name + ".preview.png")
        if os.path.isfile(MAP_PNG_PATH):
            shutil.copyfile(MAP_PNG_PATH, preview_png_path)

        snapshot = get_state_snapshot()
        map_info = snapshot.get("map_info") or {
            "width": int(w),
            "height": int(h),
            "resolution": float(msg.info.resolution),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
            "frame_id": self.map_frame,
        }
        render_info = snapshot.get("render_info") or {
            "width_cells": int(w),
            "height_cells": int(h),
            "resolution": float(msg.info.resolution),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
            "pad_left_cells": 0,
            "pad_bottom_cells": 0,
        }

        metadata = {
            "format": "slam-live-map-bundle-v1",
            "base_name": base_name,
            "map_info": map_info,
            "render_info": render_info,
            "files": {
                "preview": "preview.png",
                "occupancy": base_name + ".pgm",
                "metadata_yaml": base_name + ".yaml",
            },
        }

        bundle_json_path = os.path.join(MAP_SAVE_DIR, base_name + ".bundle.json")
        with open(bundle_json_path, "w", encoding="utf-8") as f:
            json.dump(metadata, f, ensure_ascii=False, indent=2)

        bundle_zip_path = os.path.join(MAP_SAVE_DIR, base_name + ".bundle.zip")
        with zipfile.ZipFile(bundle_zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            zf.writestr(
                "metadata.json",
                json.dumps(metadata, ensure_ascii=False, indent=2).encode("utf-8"),
            )
            if os.path.isfile(preview_png_path):
                zf.write(preview_png_path, arcname="preview.png")
            zf.write(pgm_path, arcname=os.path.basename(pgm_path))
            zf.write(yaml_path, arcname=os.path.basename(yaml_path))

    def publish_path(self, path_xy, frame_id: str):
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

        self.get_logger().info(
            f"Published path with {len(path_msg.poses)} poses to /a_star_path"
        )

    def path_xy_to_display_points(self, path_xy, goal_xy=None):
        pts = [{"x": float(x), "y": float(y)} for (x, y) in path_xy]

        if not pts or goal_xy is None:
            return pts

        gx, gy = float(goal_xy[0]), float(goal_xy[1])
        last = pts[-1]
        gap = math.hypot(last["x"] - gx, last["y"] - gy)

        # If the planner snapped the goal to a nearby free cell, keep the
        # visual path connected to the user-selected goal.
        if gap > 1e-6 and gap <= max(0.30, self.safe_clearance_m * 3.0):
            pts.append({"x": gx, "y": gy})

        return pts

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
        extra = max(4, int(0.03 * side))
        side2 = side + 2 * extra

        pad_left = (side2 - w) // 2
        pad_bottom = (side2 - h) // 2

        rgb = np.zeros((side2, side2, 3), dtype=np.float32)
        rgb[:] = [0.32, 0.32, 0.32]

        map_rgb = np.zeros((h, w, 3), dtype=np.float32)
        map_rgb[:] = [0.60, 0.60, 0.60]
        map_rgb[data == 0] = [0.95, 0.95, 0.95]
        map_rgb[data > 50] = [0.02, 0.02, 0.02]

        rgb[pad_bottom:pad_bottom + h, pad_left:pad_left + w, :] = map_rgb
        rgb = np.flipud(rgb)

        tmp_path = os.path.join(BASE_DIR, "map_tmp.png")

        try:
            img_u8 = (np.clip(rgb, 0.0, 1.0) * 255).astype(np.uint8)
            Image.fromarray(img_u8).save(tmp_path)

            if not os.path.exists(tmp_path):
                self.get_logger().error(f"map tmp png was not created: {tmp_path}")
                return

            os.replace(tmp_path, MAP_PNG_PATH)

        except Exception as e:
            self.get_logger().error(f"failed to render map png: {e}")
            return

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
            state["status"]["map_age_sec"] = age_of(self.map_last_stamp_wall)
            state["status"]["pose_age_sec"] = age_of(self.pose_last_stamp_wall)
            state["status"]["last_update"] = tnow

        update_shared_state(_upd_age)

    def fast_tick(self):
        self.process_clear_request_if_any()
        self.process_save_request_if_any()
        self.process_initial_pose_request_if_any()
        self.process_goal_request_if_any()
        self.update_pose_and_status()

    def slow_tick(self):
        self.render_map_png_if_needed()
        self.refresh_active_path()

