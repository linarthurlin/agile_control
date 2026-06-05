#!/usr/bin/env python3

"""
Waypoint Recorder & Navigator for ROS2 Humble + Nav2
-----------------------------------------------------
RECORD MODE:
  - Subscribes to /Odometry
  - Saves x, y, yaw position every N meters to waypoints.txt
  - Press 'S' + Enter to stop recording and save

PLAYBACK MODE:
  - Press 'P' + Enter to start playback
  - Reads waypoints.txt and sends all remaining points in a single NavigateThroughPoses goal
  - Nav2 plans a single continuous path through all points smoothly
"""

import os
import math
import threading
from typing import List, Optional, Tuple, Any

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses


class Waypoint:
    def __init__(self, x, y, yaw):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)


class WaypointReplanningNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_replanning_navigator')

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('spacing_m', 1.0)
        self.declare_parameter('waypoints_file', '/tmp/waypoints.txt')
        self.declare_parameter('navigate_through_poses_action', 'navigate_through_poses')
        self.declare_parameter('navigate_timeout_sec', 600.0)
        self.declare_parameter('start_from_nearest', True)
        self.declare_parameter('use_tangent_yaw_for_target', True)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.spacing_m = float(self.get_parameter('spacing_m').value)
        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.navigate_through_poses_action = self.get_parameter('navigate_through_poses_action').value
        self.navigate_timeout = float(self.get_parameter('navigate_timeout_sec').value)
        self.start_from_nearest = bool(self.get_parameter('start_from_nearest').value)
        self.use_tangent_yaw_for_target = bool(self.get_parameter('use_tangent_yaw_for_target').value)

        # ------------------------------------------------------------
        # State
        # ------------------------------------------------------------
        self.state = 'RECORDING'  # RECORDING | IDLE | NAVIGATING
        self.frame_id: Optional[str] = None
        self.waypoints: List[Waypoint] = []
        self.last_wp_pos = None
        self.latest_odom: Optional[Odometry] = None

        self.nav_thread: Optional[threading.Thread] = None
        self.nav_cancel_requested = False
        self._active_follow_goal_handle = None
        self.current_wp_idx = 0  # monotonic forward progress tracker

        # ------------------------------------------------------------
        # Subscriber
        # ------------------------------------------------------------
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

        # ------------------------------------------------------------
        # Action client
        # ------------------------------------------------------------
        self.navigate_through_poses_client = ActionClient(
            self,
            NavigateThroughPoses,
            self.navigate_through_poses_action
        )

        # ------------------------------------------------------------
        # Keyboard thread
        # ------------------------------------------------------------
        self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.kb_thread.start()

        self.get_logger().info(
            '\n'
            + '=' * 78 + '\n'
            + '  Waypoint Recorder + NavigateThroughPoses Player\n'
            + '=' * 78 + '\n'
            + f'  State                    : RECORDING\n'
            + f'  Odom topic               : {self.odom_topic}\n'
            + f'  Spacing                  : {self.spacing_m} m\n'
            + f'  Waypoints file           : {self.waypoints_file}\n'
            + f'  NavigateThroughPoses act : {self.navigate_through_poses_action}\n'
            + f'  Timeout                  : {self.navigate_timeout} s\n'
            + '=' * 78 + '\n'
            + '  Drive the vehicle to record waypoints.\n'
            + '  Press S + Enter to stop recording and save.\n'
            + '  Press P + Enter to play using NavigateThroughPoses.\n'
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
            self.get_logger().warn(
                'Odometry header.frame_id is empty. Not recording this point.'
            )
            return

        if self.frame_id is None:
            self.frame_id = current_frame
            self.get_logger().info(
                f'[REC] Waypoint frame_id set to "{self.frame_id}"'
            )

        if current_frame != self.frame_id:
            self.get_logger().error(
                f'[REC] Odometry frame changed from "{self.frame_id}" '
                f'to "{current_frame}". Stopping recording.'
            )
            self.state = 'IDLE'
            return

        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        yaw = self.yaw_from_quat(q.x, q.y, q.z, q.w)

        if self.last_wp_pos is None:
            self.record_waypoint(x, y, yaw)
            return

        dist = math.hypot(x - self.last_wp_pos[0], y - self.last_wp_pos[1])

        if dist >= self.spacing_m:
            self.record_waypoint(x, y, yaw)

    def record_waypoint(self, x, y, yaw):
        wp = Waypoint(x, y, yaw)
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
                self.nav_cancel_requested = True
                rclpy.shutdown()
                break

    # ------------------------------------------------------------
    # Save / Load
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
                f.write('# Waypoint file for Nav2 replanning playback\n')
                f.write(f'# frame_id: {self.frame_id}\n')
                f.write('# columns: x y yaw\n')

                for wp in self.waypoints:
                    f.write(
                        f'{wp.x:.6f} {wp.y:.6f} {wp.yaw:.9f}\n'
                    )

            self.get_logger().info(
                f'[SAVE] Saved {len(self.waypoints)} waypoints to "{self.waypoints_file}".'
            )
            self.get_logger().info('Press P + Enter to start playback.')

        except OSError as e:
            self.get_logger().error(f'Failed to write waypoint file: {e}')

    def load_waypoints_txt(self) -> bool:
        if not os.path.isfile(self.waypoints_file):
            self.get_logger().error(
                f'Waypoint file not found: "{self.waypoints_file}"'
            )
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

                    if len(parts) != 3:
                        self.get_logger().warn(
                            f'Skipping malformed waypoint line: {line}'
                        )
                        continue

                    x, y, yaw = map(float, parts)
                    loaded_wps.append(Waypoint(x, y, yaw))

        except (OSError, ValueError) as e:
            self.get_logger().error(f'Failed to load waypoint file: {e}')
            return False

        if loaded_frame is None:
            self.get_logger().error(
                'Waypoint file has no "# frame_id: ..." line.'
            )
            return False

        if len(loaded_wps) < 2:
            self.get_logger().error('Need at least 2 waypoints.')
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

        self.nav_cancel_requested = False
        self.current_wp_idx = 0

        self.nav_thread = threading.Thread(
            target=self.replanning_playback_loop,
            daemon=True
        )
        self.nav_thread.start()

    def replanning_playback_loop(self):
        self.state = 'NAVIGATING'

        self.get_logger().info(
            f'[NAV] Waiting for NavigateThroughPoses action server: {self.navigate_through_poses_action}'
        )
        if not self.navigate_through_poses_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f'NavigateThroughPoses action server "{self.navigate_through_poses_action}" not available.'
            )
            self.state = 'IDLE'
            return

        if self.latest_odom is None:
            self.get_logger().error('[NAV] No odometry yet. Cannot navigate.')
            self.state = 'IDLE'
            return

        nearest_idx = self.find_nearest_waypoint_index(self.waypoints)
        if nearest_idx is None:
            self.get_logger().error('[NAV] Could not find nearest waypoint index. Aborting.')
            self.state = 'IDLE'
            return

        # Prepare poses from the nearest index to the end
        remaining_waypoints = self.waypoints[nearest_idx:]
        if len(remaining_waypoints) < 1:
            self.get_logger().warn('[NAV] No remaining waypoints to follow.')
            self.state = 'IDLE'
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = []
        for i in range(len(remaining_waypoints)):
            actual_idx = nearest_idx + i
            target_pose = self.make_target_pose(actual_idx)
            goal_msg.poses.append(target_pose)

        self.get_logger().info(
            f'[NAV] Sending {len(goal_msg.poses)} waypoints to NavigateThroughPoses...'
        )

        send_future = self.navigate_through_poses_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )

        ok, goal_handle = self.wait_for_future_no_spin(
            future=send_future,
            timeout_sec=10.0,
            description='send NavigateThroughPoses goal'
        )

        if not ok or goal_handle is None:
            self.state = 'IDLE'
            return

        if not goal_handle.accepted:
            self.get_logger().error('[NAV] NavigateThroughPoses goal rejected.')
            self.state = 'IDLE'
            return

        self._active_follow_goal_handle = goal_handle
        self.get_logger().info('[NAV] Goal accepted. Navigating through waypoints...')

        result_future = goal_handle.get_result_async()
        ok, result_msg = self.wait_for_future_no_spin(
            future=result_future,
            timeout_sec=self.navigate_timeout,
            description='NavigateThroughPoses result'
        )

        self._active_follow_goal_handle = None

        if not ok:
            self.get_logger().warn('[NAV] NavigateThroughPoses timed out or cancelled.')
            try:
                goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'[NAV] Cancel failed: {e}')
        elif result_msg is None:
            self.get_logger().error('[NAV] Received empty result message.')
        elif result_msg.status == 4:
            self.get_logger().info('[NAV] All waypoints successfully reached!')
        else:
            self.get_logger().warn(f'[NAV] Navigation ended with status={result_msg.status}')

        self.state = 'IDLE'

    def nav_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[NAV] Waypoints remaining: {fb.number_of_poses_remaining}',
            throttle_duration_sec=2.0
        )

    # ------------------------------------------------------------
    # Pose helpers
    # ------------------------------------------------------------
    def make_current_pose_stamped(self) -> Optional[PoseStamped]:
        if self.latest_odom is None:
            return None

        odom_frame = self.latest_odom.header.frame_id.strip()

        if odom_frame != self.frame_id:
            self.get_logger().warn(
                f'[POSE] Latest odom frame "{odom_frame}" != waypoint frame "{self.frame_id}".'
            )
            return None

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose = self.latest_odom.pose.pose

        return pose

    def make_target_pose(self, target_idx: int) -> PoseStamped:
        wp = self.waypoints[target_idx]

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id

        pose.pose.position.x = wp.x
        pose.pose.position.y = wp.y
        pose.pose.position.z = 0.0

        if self.use_tangent_yaw_for_target:
            yaw = self.compute_tangent_yaw(self.waypoints, target_idx)
        else:
            yaw = wp.yaw

        qx, qy, qz, qw = self.quat_from_yaw(yaw)

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def find_nearest_waypoint_index(
        self,
        waypoints: List[Waypoint]
    ) -> Optional[int]:
        if self.latest_odom is None:
            self.get_logger().warn('No latest odometry.')
            return None

        odom_frame = self.latest_odom.header.frame_id.strip()

        if odom_frame != self.frame_id:
            self.get_logger().warn(
                f'Latest odom frame "{odom_frame}" != waypoint frame "{self.frame_id}". '
                f'Cannot safely find nearest waypoint.'
            )
            return None

        rx = self.latest_odom.pose.pose.position.x
        ry = self.latest_odom.pose.pose.position.y

        # Only search from a small window behind current_wp_idx forward.
        # This enforces monotonic progress and prevents jumping back to a
        # waypoint behind the robot on curved or L-shaped paths.
        search_start = max(0, self.current_wp_idx - 2)
        best_idx = search_start
        best_dist = float('inf')

        for i in range(search_start, len(waypoints)):
            d = math.hypot(waypoints[i].x - rx, waypoints[i].y - ry)

            if d < best_dist:
                best_dist = d
                best_idx = i

        self.current_wp_idx = best_idx

        self.get_logger().info(
            f'[NAV] Current odom x={rx:.3f}, y={ry:.3f}. '
            f'Nearest waypoint index={best_idx}, distance={best_dist:.3f} m.'
        )

        return best_idx

    # ------------------------------------------------------------
    # Future helper
    # ------------------------------------------------------------
    def wait_for_future_no_spin(
        self,
        future,
        timeout_sec: float,
        description: str
    ) -> Tuple[bool, Any]:
        """
        Wait for a ROS future from a worker thread.

        Important:
          - The main thread must be running rclpy.spin(node).
          - Do not call rclpy.spin_until_future_complete() here.
        """

        event = threading.Event()

        def _done_callback(_future):
            event.set()

        future.add_done_callback(_done_callback)

        finished = event.wait(timeout=timeout_sec)

        if not finished:
            self.get_logger().warn(
                f'Timeout while waiting for {description}.'
            )
            return False, None

        try:
            return True, future.result()
        except Exception as e:
            self.get_logger().error(
                f'Exception while waiting for {description}: {e}'
            )
            return False, None

    # ------------------------------------------------------------
    # Math helpers
    # ------------------------------------------------------------
    @staticmethod
    def compute_tangent_yaw(waypoints: List[Waypoint], i: int) -> float:
        if len(waypoints) < 2:
            return waypoints[i].yaw

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
    node = WaypointReplanningNavigator()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            node.nav_cancel_requested = True
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
