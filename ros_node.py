#!/usr/bin/env python3
# coding=utf-8

import json
import math
import os
import shutil
import zipfile
from PIL import Image
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
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
from shared_state import (
    set_raw_map_snapshot,
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
        self.plan_topic = os.environ.get("PLAN_TOPIC", "/plan")
        self.received_plan_topic = os.environ.get(
            "RECEIVED_PLAN_TOPIC",
            "/received_global_plan",
        )
        self.base_frame = os.environ.get("BASE_FRAME", "base_link")
        self.enable_scan_points = os.environ.get("ENABLE_SCAN_POINTS", "0") == "1"
        self.scan_stride = int(os.environ.get("SCAN_STRIDE", "4"))
        self.scan_max_points = int(os.environ.get("SCAN_MAX_POINTS", "180"))
        self.path_display_stride = max(1, int(os.environ.get("PATH_DISPLAY_STRIDE", "2")))
        self.path_display_max_points = max(2, int(os.environ.get("PATH_DISPLAY_MAX_POINTS", "300")))
        self.fast_timer_sec = float(os.environ.get("FAST_TIMER_SEC", "1.0"))
        self.slow_timer_sec = float(os.environ.get("SLOW_TIMER_SEC", "5.0"))
        self.render_map_png = os.environ.get("RENDER_MAP_PNG", "0") == "1"
        
#        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_vel_nav_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.cmd_vel_safe_pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)

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
        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            "/compute_path_to_pose",
        )
        self._nav_goal_handle = None
        self._nav_goal_future = None
        self._nav_result_future = None
        self._compute_path_goal_future = None
        self._compute_path_result_future = None

        self.local_plan_topic = os.environ.get("LOCAL_PLAN_TOPIC", "/local_plan")
        self.create_subscription(
            Path,
            self.local_plan_topic,
            self.cb_local_plan,
            10,
        )

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

        self.create_subscription(
            Path,
            self.plan_topic,
            self.cb_nav_plan,
            10,
        )
        self.create_subscription(
            Path,
            self.received_plan_topic,
            self.cb_received_plan,
            10,
        )

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )
        self.goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.get_logger().info(
            f"[slam_live_map] HTTP: http://<IP_RPI>:8080/ | map={self.map_topic} | scan={self.scan_topic} | plan={self.plan_topic} | received_plan={self.received_plan_topic} | base_frame={self.base_frame}"
        )

        self.fast_timer = self.create_timer(self.fast_timer_sec, self.fast_tick)
        self.slow_timer = self.create_timer(self.slow_timer_sec, self.slow_tick)
    def publish_stop_cmd(self):
        msg = Twist()

        for _ in range(5):
            self.cmd_vel_pub.publish(msg)
            self.cmd_vel_nav_pub.publish(msg)
            self.cmd_vel_safe_pub.publish(msg)
    def cb_local_plan(self, msg: Path):
        points = self.downsample_points([
            {"x": float(p.pose.position.x), "y": float(p.pose.position.y)}
            for p in msg.poses
        ])

        def _upd(state):
            state["paths"]["local_plan"] = points
            state["status"]["last_update"] = now_sec()

        update_shared_state(_upd)
    def update_nav_path_display(self, path_xy, source: str, path_key: str = "plan"):
        full_points = [{"x": float(x), "y": float(y)} for x, y in (path_xy or [])]
        points = self.downsample_points(full_points)
        path_key = path_key if path_key in ("plan", "received_plan") else "plan"

        if path_key == "received_plan" or not self.path_xy:
            self.path_xy = [(p["x"], p["y"]) for p in full_points]

        def _upd(state):
            state["paths"][path_key] = points
            if path_key == "received_plan":
                state["paths"]["nav2"] = points
                state["paths"]["a_star"] = points
            elif not state["paths"].get("received_plan"):
                state["paths"]["nav2"] = points
                state["paths"]["a_star"] = points
            state["status"]["planner_ok"] = bool(points)
            state["status"]["planner_msg"] = (
                f"{source} ({len(points)} pts)" if points else f"{source} empty"
            )
            state["status"]["last_update"] = now_sec()

        update_shared_state(_upd)

    def downsample_points(self, points):
        if not points:
            return []
        if len(points) <= self.path_display_max_points and self.path_display_stride <= 1:
            return points

        sampled = points[:: self.path_display_stride]
        if sampled[-1] != points[-1]:
            sampled.append(points[-1])

        if len(sampled) <= self.path_display_max_points:
            return sampled

        step = max(1, math.ceil(len(sampled) / self.path_display_max_points))
        capped = sampled[::step]
        if capped[-1] != sampled[-1]:
            capped.append(sampled[-1])
        return capped[: self.path_display_max_points - 1] + [sampled[-1]]

    def cb_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.map_frame = msg.header.frame_id or "map"
        self.map_last_stamp_wall = now_sec()

        h = msg.info.height
        w = msg.info.width
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)

        occ_count = int(np.count_nonzero(data > 50))
        free_count = int(np.count_nonzero(data == 0))

        try:
            fp = (
                int(w),
                int(h),
                float(msg.info.resolution),
                float(msg.info.origin.position.x),
                float(msg.info.origin.position.y),
                occ_count,
                free_count,
            )
        except Exception:
            fp = (int(w), int(h), now_sec())

        map_changed = fp != self.last_map_fingerprint
        if map_changed:
            self.last_map_fingerprint = fp
            self.map_dirty = True

        side = max(int(w), int(h))
        extra = max(4, int(0.03 * side))
        side2 = side + 2 * extra
        pad_left = (side2 - int(w)) // 2
        pad_bottom = (side2 - int(h)) // 2
        res = float(msg.info.resolution)
        render_origin_x = float(msg.info.origin.position.x) - pad_left * res
        render_origin_y = float(msg.info.origin.position.y) - pad_bottom * res

        next_map_version = None
        if map_changed:
            snapshot = get_state_snapshot()
            next_map_version = int(snapshot.get("map_version") or 0) + 1
            set_raw_map_snapshot(
                width=int(w),
                height=int(h),
                resolution=res,
                origin_x=float(msg.info.origin.position.x),
                origin_y=float(msg.info.origin.position.y),
                frame_id=self.map_frame,
                data=data.astype(np.int8, copy=False).tobytes(),
                map_version=next_map_version,
            )

        def _upd(state):
            if map_changed and next_map_version is not None:
                state["map_version"] = next_map_version
            state["map_info"] = {
                "width": int(w),
                "height": int(h),
                "resolution": res,
                "origin_x": float(msg.info.origin.position.x),
                "origin_y": float(msg.info.origin.position.y),
                "frame_id": self.map_frame,
            }
            state["render_info"] = {
                "width_cells": int(side2),
                "height_cells": int(side2),
                "resolution": res,
                "origin_x": float(render_origin_x),
                "origin_y": float(render_origin_y),
                "pad_left_cells": int(pad_left),
                "pad_bottom_cells": int(pad_bottom),
            }
            state["status"]["slam_ok"] = True
            state["status"]["last_update"] = now_sec()

        update_shared_state(_upd)

    def cb_scan(self, msg: LaserScan):
        self.scan_last_stamp_wall = now_sec()

        stamp_sec = (
            float(msg.header.stamp.sec) +
            float(msg.header.stamp.nanosec) * 1e-9
        )

        samples = []
        angle = msg.angle_min
        stride = max(1, self.scan_stride)

        for i, rng in enumerate(msg.ranges):
            if i % stride == 0 and math.isfinite(rng):
                if msg.range_min <= rng <= msg.range_max:
                    samples.append({
                        "angle": float(angle),
                        "range": float(rng),
                    })
                    if len(samples) >= self.scan_max_points:
                        break

            angle += msg.angle_increment

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            def _upd_fail(state):
                state["scan"]["points"] = []
                state["scan"]["raw"] = {
                    "samples": samples,
                    "range_min": float(msg.range_min),
                    "range_max": float(msg.range_max),
                }
                state["scan"]["ok"] = False
                state["scan"]["stamp"] = stamp_sec
                state["scan"]["frame_id"] = msg.header.frame_id
                state["status"]["last_update"] = now_sec()
            update_shared_state(_upd_fail)
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        yaw = quat_to_yaw(tf.transform.rotation)

        points = []
        if self.enable_scan_points:
            cy = math.cos(yaw)
            sy = math.sin(yaw)
            for sample in samples:
                rng = sample["range"]
                angle = sample["angle"]
                lx = rng * math.cos(angle)
                ly = rng * math.sin(angle)
                wx = tx + cy * lx - sy * ly
                wy = ty + sy * lx + cy * ly
                points.append({"x": float(wx), "y": float(wy)})

        def _upd(state):
            state["scan"]["points"] = points
            state["scan"]["raw"] = {
                "samples": samples,
                "range_min": float(msg.range_min),
                "range_max": float(msg.range_max),
            }
            state["scan"]["transform"] = {
                "x": float(tx),
                "y": float(ty),
                "yaw": float(yaw),
            }
            state["scan"]["stamp"] = stamp_sec
            state["scan"]["frame_id"] = msg.header.frame_id
            state["scan"]["ok"] = True
            state["status"]["last_update"] = now_sec()

        update_shared_state(_upd)

    def cb_nav_plan(self, msg: Path):
        path_xy = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
        ]
        self.update_nav_path_display(
            path_xy,
            f"nav2 topic {self.plan_topic}",
            path_key="plan",
        )

    def cb_received_plan(self, msg: Path):
        path_xy = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in msg.poses
        ]
        self.update_nav_path_display(
            path_xy,
            f"nav2 topic {self.received_plan_topic}",
            path_key="received_plan",
        )

    def make_goal_pose_msg(self, gx: float, gy: float, goal_yaw: float):
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.position.z = 0.0

        qz, qw = yaw_to_quat(goal_yaw)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def publish_goal_pose_topic(self, gx: float, gy: float, goal_yaw: float):
        msg = self.make_goal_pose_msg(gx, gy, goal_yaw)
        self.goal_pose_pub.publish(msg)
        self.get_logger().info(
            f"Published /goal_pose: x={gx:.2f}, y={gy:.2f}, yaw={math.degrees(goal_yaw):.1f} deg"
        )

    def send_nav2_goal(self, gx: float, gy: float, goal_yaw: float):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("NavigateToPose action server not available")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_goal_pose_msg(gx, gy, goal_yaw)

        self.get_logger().info(
            f"Sending Nav2 goal: x={gx:.2f}, y={gy:.2f}, yaw={math.degrees(goal_yaw):.1f} deg"
        )

        self._nav_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self._nav_goal_future.add_done_callback(self._nav_goal_response_callback)
        return True

    def request_nav2_plan(self, gx: float, gy: float, goal_yaw: float):
        if not self.compute_path_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("ComputePathToPose action server not available")

            def _upd(state):
                state["status"]["planner_msg"] = (
                    f"nav2 goal sent, waiting for path topic {self.plan_topic}"
                )
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd)
            return False

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal.header.frame_id = self.map_frame
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = gx
        goal_msg.goal.pose.position.y = gy
        goal_msg.goal.pose.position.z = 0.0

        qz, qw = yaw_to_quat(goal_yaw)
        goal_msg.goal.pose.orientation.z = qz
        goal_msg.goal.pose.orientation.w = qw

        if hasattr(goal_msg, "planner_id"):
            goal_msg.planner_id = ""
        if hasattr(goal_msg, "use_start"):
            goal_msg.use_start = False

        self._compute_path_goal_future = self.compute_path_client.send_goal_async(goal_msg)
        self._compute_path_goal_future.add_done_callback(
            self._compute_path_goal_response_callback
        )
        return True

    def _compute_path_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().warn(f"ComputePathToPose send failed: {e}")

            def _upd(state):
                state["status"]["planner_msg"] = f"nav2 path request failed: {e}"
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd)
            return

        if not goal_handle.accepted:
            self.get_logger().warn("ComputePathToPose request was rejected")

            def _upd(state):
                state["status"]["planner_msg"] = "nav2 path request rejected"
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd)
            return

        self._compute_path_result_future = goal_handle.get_result_async()
        self._compute_path_result_future.add_done_callback(
            self._compute_path_result_callback
        )

    def _compute_path_result_callback(self, future):
        try:
            result = future.result().result
            path_msg = getattr(result, "path", None)
            path_xy = []
            if path_msg is not None:
                path_xy = [
                    (float(pose.pose.position.x), float(pose.pose.position.y))
                    for pose in path_msg.poses
                ]

            if path_xy:
                self.update_nav_path_display(path_xy, "nav2 computed path")
                return

            error_msg = getattr(result, "error_msg", "") or "empty path"
            self.get_logger().warn(f"ComputePathToPose returned no path: {error_msg}")

            def _upd(state):
                state["paths"]["nav2"] = []
                state["paths"]["a_star"] = []
                state["status"]["planner_ok"] = False
                state["status"]["planner_msg"] = f"nav2 path empty: {error_msg}"
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd)

        except Exception as e:
            self.get_logger().warn(f"ComputePathToPose result failed: {e}")

            def _upd(state):
                state["status"]["planner_ok"] = False
                state["status"]["planner_msg"] = f"nav2 path result error: {e}"
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd)

    def _nav_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self._nav_goal_handle = None
            self.get_logger().error(f"Nav2 goal send failed: {e}")
            return

        if not goal_handle.accepted:
            self._nav_goal_handle = None
            self.get_logger().warn("Nav2 goal was rejected")

            def _upd(state):
                state["status"]["planner_ok"] = False
                state["status"]["planner_msg"] = "nav2 goal rejected"
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd)
            return

        self.get_logger().info("Nav2 goal accepted")
        self._nav_goal_handle = goal_handle
        self._nav_result_future = goal_handle.get_result_async()
        self._nav_result_future.add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        self._nav_goal_handle = None
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
        if req is None:
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
        if req is None:
            return

        goal_yaw = req["yaw"]
        requested_gx = float(req["x"])
        requested_gy = float(req["y"])

        def _upd_goal_requested(state):
            state["goal"]["x"] = float(requested_gx)
            state["goal"]["y"] = float(requested_gy)
            state["goal"]["yaw"] = float(goal_yaw)
            state["goal"]["stamp"] = now_sec()
            state["paths"]["nav2"] = []
            state["paths"]["a_star"] = []
            state["status"]["planner_ok"] = False
            state["status"]["planner_msg"] = "sending nav2 goal..."
            state["status"]["last_update"] = now_sec()

        update_shared_state(_upd_goal_requested)

        if self.send_nav2_goal(requested_gx, requested_gy, goal_yaw):
            # self.publish_goal_pose_topic(requested_gx, requested_gy, goal_yaw)

            def _upd_sent(state):
                state["status"]["planner_msg"] = "nav2 goal sent, requesting path..."
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd_sent)
            self.request_nav2_plan(requested_gx, requested_gy, goal_yaw)
        else:
            def _upd_fail(state):
                state["status"]["planner_ok"] = False
                state["status"]["planner_msg"] = "nav2 action server unavailable"
                state["status"]["last_update"] = now_sec()

            update_shared_state(_upd_fail)

    def process_clear_request_if_any(self):
        if not pop_clear_request():
            return

        self.path_xy = None

        if self._nav_goal_handle is not None:
            try:
                self._nav_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"Cancel nav2 goal failed: {e}")

        self._nav_goal_handle = None
        self.publish_stop_cmd()

        def _upd(state):
            state["goal"]["x"] = None
            state["goal"]["y"] = None
            state["goal"]["yaw"] = None
            state["goal"]["stamp"] = now_sec()
            state["paths"]["nav2"] = []
            state["paths"]["a_star"] = []
            state["paths"]["plan"] = []
            state["paths"]["received_plan"] = []
            state["paths"]["local_plan"] = []

            state["status"]["planner_ok"] = False
            state["status"]["planner_msg"] = "stopped and cleared"
            state["status"]["last_update"] = now_sec()

        update_shared_state(_upd)
        self.get_logger().info("Clear path request: nav2 goal cancel requested.")

    def process_save_request_if_any(self):
        name = pop_save_request()
        if not name:
            return
        if self.map_msg is None:
            self.get_logger().warn("SAVE_MAP: map_msg is None, cannot save.")
            return

        try:
            if self.render_map_png:
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
        if self.render_map_png:
            self.render_map_png_if_needed()
