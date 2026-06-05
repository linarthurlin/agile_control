#!/usr/bin/env python3
"""
Waypoint Recorder & Navigator for ROS2 Humble + Nav2
-----------------------------------------------------
RECORD MODE:
  - Subscribes to /Odometry
  - Saves x,y position every N meters to waypoints.txt
  - Press 'S' + Enter to stop recording and save

PLAYBACK MODE:
  - Press 'P' + Enter to start playback
  - Reads waypoints.txt and sends ALL points in one NavigateThroughPoses goal
  - Nav2 plans a single continuous path through all points (no stopping)

Usage:
  ros2 run <your_package> waypoint_recorder_navigator
  ros2 run <your_package> waypoint_recorder_navigator --ros-args \
      -p spacing_m:=2.0 -p waypoints_file:=/tmp/waypoints.txt
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped

import math
import threading
import os


class WaypointRecorderNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_recorder_navigator')

        # ---------- Parameters ----------
        self.declare_parameter('spacing_m', 4.0)
        self.declare_parameter('waypoints_file', 'waypoints.txt')
        self.declare_parameter('goal_timeout_sec', 60.0)

        self.spacing_m      = self.get_parameter('spacing_m').value
        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.goal_timeout   = self.get_parameter('goal_timeout_sec').value

        # ---------- State ----------
        self.state        = 'RECORDING'   # RECORDING | IDLE | NAVIGATING
        self.waypoints    = []            # list of (x, y)
        self.last_wp_pos  = None          # (x, y) of last recorded waypoint

        # ---------- QoS for Odometry ----------
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # ---------- Subscriber ----------
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            odom_qos
        )

        # ---------- Nav2 Action Client ----------
        self.nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        # ---------- Keyboard thread ----------
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        self.get_logger().info(
            f'\n{"="*55}\n'
            f'  Waypoint Recorder / Navigator\n'
            f'{"="*55}\n'
            f'  State       : RECORDING\n'
            f'  Spacing     : {self.spacing_m} m\n'
            f'  Output file : {self.waypoints_file}\n'
            f'{"="*55}\n'
            f'  Drive the vehicle. Waypoints saved every {self.spacing_m} m.\n'
            f'  Press  S + Enter  to STOP recording & save file.\n'
            f'  Press  P + Enter  to start PLAYBACK (Nav2).\n'
            f'  Press  Q + Enter  to quit.\n'
        )

    # ------------------------------------------------------------------
    # Odometry callback
    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.state != 'RECORDING':
            return

        if self.last_wp_pos is None:
            self._record_waypoint(x, y)
            return

        dist = math.hypot(x - self.last_wp_pos[0], y - self.last_wp_pos[1])
        if dist >= self.spacing_m:
            self._record_waypoint(x, y)

    def _record_waypoint(self, x, y):
        self.waypoints.append((x, y))
        self.last_wp_pos = (x, y)
        self.get_logger().info(
            f'[REC] Waypoint #{len(self.waypoints):4d}  '
            f'x={x:8.3f}  y={y:8.3f}'
        )

    # ------------------------------------------------------------------
    # Keyboard input – blocking readline in a daemon thread
    # ------------------------------------------------------------------
    def _keyboard_loop(self):
        while rclpy.ok():
            try:
                key = input().strip().upper()
            except EOFError:
                break

            if key == 'S':
                self._stop_recording()
            elif key == 'P':
                self._start_playback()
            elif key == 'Q':
                self.get_logger().info('Quit requested.')
                rclpy.shutdown()
                break

    # ------------------------------------------------------------------
    # Stop recording & save
    # ------------------------------------------------------------------
    def _stop_recording(self):
        if self.state != 'RECORDING':
            self.get_logger().warn('Not currently recording.')
            return

        self.state = 'IDLE'
        self.get_logger().info(
            f'[STOP] Recording stopped. {len(self.waypoints)} waypoints collected.'
        )

        if not self.waypoints:
            self.get_logger().warn('No waypoints recorded – file not written.')
            return

        try:
            with open(self.waypoints_file, 'w') as f:
                f.write('# x, y\n')
                for (x, y) in self.waypoints:
                    f.write(f'{x:.6f}, {y:.6f}\n')
            self.get_logger().info(
                f'[SAVE] {len(self.waypoints)} waypoints saved to '
                f'"{self.waypoints_file}".\n'
                f'       Press  P + Enter  to start Nav2 playback.'
            )
        except OSError as e:
            self.get_logger().error(f'Failed to write file: {e}')

    # ------------------------------------------------------------------
    # Start playback
    # ------------------------------------------------------------------
    def _start_playback(self):
        if self.state == 'NAVIGATING':
            self.get_logger().warn('Already navigating.')
            return

        if self.state == 'RECORDING':
            self.get_logger().warn(
                'Still recording. Press  S + Enter  first to stop & save.'
            )
            return

        if not self._load_waypoints():
            return

        self.state = 'NAVIGATING'
        self.get_logger().info(
            f'[NAV] Starting playback of {len(self.waypoints)} waypoints …'
        )
        nav_thread = threading.Thread(target=self._navigate_waypoints, daemon=True)
        nav_thread.start()

    def _load_waypoints(self):
        if not os.path.isfile(self.waypoints_file):
            self.get_logger().error(
                f'Waypoints file not found: "{self.waypoints_file}"'
            )
            return False

        waypoints = []
        try:
            with open(self.waypoints_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    parts = line.split(',')
                    if len(parts) < 2:
                        continue
                    waypoints.append((float(parts[0]), float(parts[1])))
        except (OSError, ValueError) as e:
            self.get_logger().error(f'Failed to read waypoints file: {e}')
            return False

        if not waypoints:
            self.get_logger().error('Waypoints file is empty.')
            return False

        self.waypoints = waypoints
        self.get_logger().info(
            f'[LOAD] {len(self.waypoints)} waypoints loaded from '
            f'"{self.waypoints_file}".'
        )
        return True

    # ------------------------------------------------------------------
    # Send all waypoints in a single NavigateThroughPoses goal
    # ------------------------------------------------------------------
    def _navigate_waypoints(self):
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'navigate_through_poses action server not available! Is Nav2 running?'
            )
            self.state = 'IDLE'
            return

        total = len(self.waypoints)
        self.get_logger().info(
            f'[NAV] Sending all {total} waypoints as a single NavigateThroughPoses goal …'
        )

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [self._make_pose_stamped(x, y) for x, y in self.waypoints]

        # Send goal
        send_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        if not send_future.done():
            self.get_logger().error('Goal send timed out. Aborting playback.')
            self.state = 'IDLE'
            return

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2. Aborting playback.')
            self.state = 'IDLE'
            return

        self.get_logger().info('[NAV] Goal accepted. Navigating through all waypoints …')

        # Wait for the single result (vehicle has reached the final pose)
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self.goal_timeout
        )

        if not result_future.done():
            self.get_logger().warn(
                f'Navigation timed out after {self.goal_timeout}s. Cancelling.'
            )
            goal_handle.cancel_goal_async()
            self.state = 'IDLE'
            return

        # action_msgs/GoalStatus SUCCEEDED = 4
        status = result_future.result().status
        if status == 4:
            self.get_logger().info('[NAV] All waypoints REACHED ✓')
        else:
            self.get_logger().warn(f'[NAV] Navigation ended with status={status}.')

        self.get_logger().info('[NAV] Playback complete. State -> IDLE.')
        self.state = 'IDLE'

    def _nav_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[NAV] Waypoints remaining: {fb.number_of_poses_remaining}',
            throttle_duration_sec=2.0
        )

    # ------------------------------------------------------------------
    # Helper: PoseStamped in the odom frame with identity orientation
    # ------------------------------------------------------------------
    def _make_pose_stamped(self, x: float, y: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0   # identity – Nav2 will compute heading
        return pose


# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorderNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()