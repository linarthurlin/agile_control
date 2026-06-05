#!/usr/bin/env python3

"""
Waypoint Recorder + FollowPath Player for ROS2 Humble / Nav2

Run directly with python3:

  python3 record_followpath --ros-args \
    -p odom_topic:=/Odometry \
    -p spacing_m:=2.0 \
    -p waypoints_file:=waypoints.txt \
    -p follow_path_action:=/follow_path \
    -p controller_id:=FollowPath \
    -p goal_checker_id:=goal_checker \
    -p start_from_nearest:=true \
    -p use_tangent_yaw:=true

Keyboard:
  S + Enter : stop recording and save txt
  P + Enter : load txt and send FollowPath
  Q + Enter : quit
"""

import os
import math
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath


class Waypoint:
    def __init__(self, x, y, yaw, qx, qy, qz, qw):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.qx = float(qx)
        self.qy = float(qy)
        self.qz = float(qz)
        self.qw = float(qw)


class WaypointFollowPathTxt(Node):

    def __init__(self):
        super().__init__('waypoint_followpath_txt')

        # ---------------- Parameters ----------------
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('spacing_m', 1.0)
        self.declare_parameter('waypoints_file', '/tmp/waypoints.txt')

        # Nav2 controller_server FollowPath action.
        # Usually: /follow_path
        self.declare_parameter('follow_path_action', '/follow_path')

        # Must match controller_plugins: ["FollowPath"]
        self.declare_parameter('controller_id', 'FollowPath')

        # Must match current_goal_checker: "goal_checker"
        self.declare_parameter('goal_checker_id', 'goal_checker')

        # Skip already-passed points by finding nearest waypoint to current odom.
        self.declare_parameter('start_from_nearest', True)

        # True: path orientation is computed from waypoint-to-waypoint direction.
        # False: use recorded odometry orientation.
        self.declare_parameter('use_tangent_yaw', True)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.spacing_m = float(self.get_parameter('spacing_m').value)
        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.follow_path_action = self.get_parameter('follow_path_action').value
        self.controller_id = self.get_parameter('controller_id').value
        self.goal_checker_id = self.get_parameter('goal_checker_id').value
        self.start_from_nearest = bool(self.get_parameter('start_from_nearest').value)
        self.use_tangent_yaw = bool(self.get_parameter('use_tangent_yaw').value)

        # ---------------- State ----------------
        self.state = 'RECORDING'  # RECORDING | IDLE | NAVIGATING
        self.frame_id: Optional[str] = None
        self.waypoints: List[Waypoint] = []
        self.last_wp_pos = None
        self.latest_odom: Optional[Odometry] = None

        # ---------------- Subscriber ----------------
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            odom_qos
        )

        # ---------------- FollowPath action client ----------------
        self.follow_client = ActionClient(
            self,
            FollowPath,
            self.follow_path_action
        )

        # ---------------- Keyboard thread ----------------
        self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.kb_thread.start()

        self.get_logger().info(
            '\n'
            + '=' * 70 + '\n'
            + '  Waypoint TXT Recorder + Nav2 FollowPath Player\n'
            + '=' * 70 + '\n'
            + f'  State              : RECORDING\n'
            + f'  Odom topic         : {self.odom_topic}\n'
            + f'  Spacing            : {self.spacing_m} m\n'
            + f'  Waypoints file     : {self.waypoints_file}\n'
            + f'  FollowPath action  : {self.follow_path_action}\n'
            + f'  Controller ID      : {self.controller_id}\n'
            + f'  Goal checker ID    : {self.goal_checker_id}\n'
            + f'  Start from nearest : {self.start_from_nearest}\n'
            + f'  Use tangent yaw    : {self.use_tangent_yaw}\n'
            + '=' * 70 + '\n'
            + '  Drive the vehicle to record waypoints.\n'
            + '  Press S + Enter to stop recording and save.\n'
            + '  Press P + Enter to play using FollowPath.\n'
            + '  Press Q + Enter to quit.\n'
        )

    # ------------------------------------------------------------
    # Odometry callback
    # ------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

        if self.state != 'RECORDING':
            return

        current_frame = msg.header.frame_id.strip()

        if current_frame == '':
            self.get_logger().warn('Odometry header.frame_id is empty. Not recording this point.')
            return

        if self.frame_id is None:
            self.frame_id = current_frame
            self.get_logger().info(f'[REC] Waypoint frame_id set to "{self.frame_id}"')

        if current_frame != self.frame_id:
            self.get_logger().error(
                f'[REC] Odometry frame changed from "{self.frame_id}" to "{current_frame}". '
                f'Stopping recording to avoid mixed-frame waypoints.'
            )
            self.state = 'IDLE'
            return

        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        yaw = self.yaw_from_quat(q.x, q.y, q.z, q.w)

        if self.last_wp_pos is None:
            self.record_waypoint(x, y, yaw, q.x, q.y, q.z, q.w)
            return

        dist = math.hypot(x - self.last_wp_pos[0], y - self.last_wp_pos[1])

        if dist >= self.spacing_m:
            self.record_waypoint(x, y, yaw, q.x, q.y, q.z, q.w)

    def record_waypoint(self, x, y, yaw, qx, qy, qz, qw):
        wp = Waypoint(x, y, yaw, qx, qy, qz, qw)
        self.waypoints.append(wp)
        self.last_wp_pos = (x, y)

        self.get_logger().info(
            f'[REC] #{len(self.waypoints):04d} '
            f'frame={self.frame_id} '
            f'x={x:8.3f} y={y:8.3f} yaw={yaw:7.3f}'
        )

    # ------------------------------------------------------------
    # Keyboard
    # ------------------------------------------------------------
    def keyboard_loop(self):
        while rclpy.ok():
            try:
                key = input().strip().upper()
            except EOFError:
                break

            if key == 'S':
                self.stop_recording_and_save()
            elif key == 'P':
                self.start_playback()
            elif key == 'Q':
                self.get_logger().info('Quit requested.')
                rclpy.shutdown()
                break

    # ------------------------------------------------------------
    # Save txt
    # ------------------------------------------------------------
    def stop_recording_and_save(self):
        if self.state != 'RECORDING':
            self.get_logger().warn('Not currently recording.')
            return

        self.state = 'IDLE'

        if not self.waypoints:
            self.get_logger().warn('No waypoints recorded. File not written.')
            return

        if self.frame_id is None:
            self.get_logger().error('No frame_id was recorded. File not written.')
            return

        try:
            with open(self.waypoints_file, 'w') as f:
                f.write('# Waypoint file for Nav2 FollowPath\n')
                f.write(f'# frame_id: {self.frame_id}\n')
                f.write('# columns: x y yaw qx qy qz qw\n')

                for wp in self.waypoints:
                    f.write(
                        f'{wp.x:.6f} {wp.y:.6f} {wp.yaw:.9f} '
                        f'{wp.qx:.9f} {wp.qy:.9f} {wp.qz:.9f} {wp.qw:.9f}\n'
                    )

            self.get_logger().info(
                f'[SAVE] Saved {len(self.waypoints)} waypoints to "{self.waypoints_file}".'
            )
            self.get_logger().info('Press P + Enter to send FollowPath.')

        except OSError as e:
            self.get_logger().error(f'Failed to write waypoint file: {e}')

    # ------------------------------------------------------------
    # Load txt
    # ------------------------------------------------------------
    def load_waypoints_txt(self) -> bool:
        if not os.path.isfile(self.waypoints_file):
            self.get_logger().error(f'Waypoint file not found: "{self.waypoints_file}"')
            return False

        loaded_frame = None
        loaded_wps: List[Waypoint] = []

        try:
            with open(self.waypoints_file, 'r') as f:
                for raw_line in f:
                    line = raw_line.strip()

                    if line == '':
                        continue

                    if line.startswith('#'):
                        if line.startswith('# frame_id:'):
                            loaded_frame = line.split(':', 1)[1].strip()
                        continue

                    parts = line.split()

                    if len(parts) != 7:
                        self.get_logger().warn(f'Skipping malformed waypoint line: {line}')
                        continue

                    x, y, yaw, qx, qy, qz, qw = map(float, parts)
                    loaded_wps.append(Waypoint(x, y, yaw, qx, qy, qz, qw))

        except (OSError, ValueError) as e:
            self.get_logger().error(f'Failed to load waypoint file: {e}')
            return False

        if loaded_frame is None:
            self.get_logger().error(
                'Waypoint file has no "# frame_id: ..." line. '
                'This is unsafe because path frame is unknown.'
            )
            return False

        if len(loaded_wps) < 2:
            self.get_logger().error('Need at least 2 waypoints for FollowPath.')
            return False

        self.frame_id = loaded_frame
        self.waypoints = loaded_wps

        self.get_logger().info(
            f'[LOAD] Loaded {len(self.waypoints)} waypoints from "{self.waypoints_file}". '
            f'frame_id="{self.frame_id}"'
        )
        return True

    # ------------------------------------------------------------
    # Playback
    # ------------------------------------------------------------
    def start_playback(self):
        if self.state == 'RECORDING':
            self.get_logger().warn('Still recording. Press S first.')
            return

        if self.state == 'NAVIGATING':
            self.get_logger().warn('Already navigating.')
            return

        if not self.load_waypoints_txt():
            return

        nav_thread = threading.Thread(target=self.send_follow_path, daemon=True)
        nav_thread.start()

    '''
    def send_follow_path(self):
        self.state = 'NAVIGATING'

        self.get_logger().info(
            f'[NAV] Waiting for FollowPath action server: {self.follow_path_action}'
        )

        if not self.follow_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f'FollowPath action server "{self.follow_path_action}" not available. '
                f'Check controller_server lifecycle state.'
            )
            self.state = 'IDLE'
            return

        path_wps = self.waypoints

        if self.start_from_nearest:
            nearest_idx = self.find_nearest_waypoint_index(path_wps)
            if nearest_idx is not None:
                start_idx = max(0, nearest_idx - 1)
                self.get_logger().info(
                    f'[NAV] Nearest waypoint index={nearest_idx}. '
                    f'Sending path from index={start_idx}.'
                )
                path_wps = path_wps[start_idx:]

        if len(path_wps) < 2:
            self.get_logger().error('Path after trimming has fewer than 2 waypoints.')
            self.state = 'IDLE'
            return

        path_msg = self.make_path_msg(path_wps)

        goal = FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id

        self.get_logger().info(
            f'[NAV] Sending FollowPath with {len(path_msg.poses)} poses. '
            f'path.frame_id="{path_msg.header.frame_id}"'
        )

        send_future = self.follow_client.send_goal_async(goal)
        #rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        send_future = self.follow_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)
        if not send_future.done():
            self.get_logger().error('Sending FollowPath goal timed out.')
            self.state = 'IDLE'
            return

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal rejected.')
            self.state = 'IDLE'
            return

        self.get_logger().info('[NAV] FollowPath accepted. Waiting for result...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        if result_future.done():
            status = result_future.result().status
            self.get_logger().info(f'[NAV] FollowPath finished. status={status}')
        else:
            self.get_logger().warn('[NAV] FollowPath result did not complete.')

        self.state = 'IDLE'
    '''
    def send_follow_path(self):
        self.state = 'NAVIGATING'

        self.get_logger().info(
            f'[NAV] Waiting for FollowPath action server: {self.follow_path_action}'
        )

        if not self.follow_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f'FollowPath action server "{self.follow_path_action}" not available.'
            )
            self.state = 'IDLE'
            return

        path_wps = self.waypoints

        if self.start_from_nearest:
            nearest_idx = self.find_nearest_waypoint_index(path_wps)
            if nearest_idx is not None:
                start_idx = max(0, nearest_idx - 1)

                self.get_logger().info(
                    f'[NAV] Nearest waypoint index={nearest_idx}. '
                    f'Sending path from index={start_idx}.'
                )

                path_wps = path_wps[start_idx:]

        if len(path_wps) < 2:
            self.get_logger().error('Path after trimming has fewer than 2 waypoints.')
            self.state = 'IDLE'
            return

        path_msg = self.make_path_msg(path_wps)

        goal = FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id

        self.get_logger().info(
            f'[NAV] Sending FollowPath with {len(path_msg.poses)} poses. '
            f'path.frame_id="{path_msg.header.frame_id}"'
        )

        send_future = self.follow_client.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)

    # ------------------------------------------------------------
    # Path construction
    # ------------------------------------------------------------
    def make_path_msg(self, waypoints: List[Waypoint]) -> Path:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id

        for i, wp in enumerate(waypoints):
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = self.frame_id

            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            pose.pose.position.z = 0.0

            if self.use_tangent_yaw:
                yaw = self.compute_tangent_yaw(waypoints, i)
                qx, qy, qz, qw = self.quat_from_yaw(yaw)
            else:
                qx, qy, qz, qw = wp.qx, wp.qy, wp.qz, wp.qw

            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path.poses.append(pose)

        return path

    def find_nearest_waypoint_index(self, waypoints: List[Waypoint]) -> Optional[int]:
        if self.latest_odom is None:
            self.get_logger().warn('No latest odometry. Using path from first waypoint.')
            return None

        odom_frame = self.latest_odom.header.frame_id.strip()

        if odom_frame != self.frame_id:
            self.get_logger().warn(
                f'Latest odom frame "{odom_frame}" != waypoint frame "{self.frame_id}". '
                f'Cannot safely find nearest waypoint. Using path from first waypoint.'
            )
            return None

        rx = self.latest_odom.pose.pose.position.x
        ry = self.latest_odom.pose.pose.position.y

        best_idx = 0
        best_dist = float('inf')

        for i, wp in enumerate(waypoints):
            d = math.hypot(wp.x - rx, wp.y - ry)
            if d < best_dist:
                best_dist = d
                best_idx = i

        self.get_logger().info(
            f'[NAV] Current odom x={rx:.3f}, y={ry:.3f}. '
            f'Nearest waypoint distance={best_dist:.3f} m.'
        )
        return best_idx

    # ------------------------------------------------------------
    # Math helpers
    # ------------------------------------------------------------
    @staticmethod
    def compute_tangent_yaw(waypoints: List[Waypoint], i: int) -> float:
        if i < len(waypoints) - 1:
            dx = waypoints[i + 1].x - waypoints[i].x
            dy = waypoints[i + 1].y - waypoints[i].y
        else:
            dx = waypoints[i].x - waypoints[i - 1].x
            dy = waypoints[i].y - waypoints[i - 1].y

        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return waypoints[i].yaw

        return math.atan2(dy, dx)

    @staticmethod
    def quat_from_yaw(yaw: float):
        half = 0.5 * yaw
        return 0.0, 0.0, math.sin(half), math.cos(half)

    @staticmethod
    def yaw_from_quat(qx, qy, qz, qw) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowPathTxt()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


