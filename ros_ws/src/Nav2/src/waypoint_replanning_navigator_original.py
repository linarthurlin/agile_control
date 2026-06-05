#!/usr/bin/env python3

"""
Waypoint Recorder + Replanning Nav2 Player for ROS2 Humble / Nav2

python3 waypoint_replanning_navigator.py --ros-args \
  -p odom_topic:=/Odometry \
  -p spacing_m:=1.0 \
  -p waypoints_file:=waypoints.txt \
  -p compute_path_action:=/compute_path_to_pose \
  -p follow_path_action:=/follow_path \
  -p planner_id:=GridBased \
  -p controller_id:=FollowPath \
  -p goal_checker_id:=goal_checker \
  -p target_lookahead_wps:=15 \
  -p follow_path_timeout_sec:=25.0


Main idea:
  RECORD:
    - Subscribe to /Odometry
    - Save waypoints to txt

  PLAYBACK:
    - Load recorded waypoint txt
    - Find nearest waypoint from current odom
    - Select a lookahead waypoint as the temporary target
    - Ask Nav2 planner_server using ComputePathToPose
    - Send the planned path to controller_server using FollowPath
    - If FollowPath fails/times out, replan from current odom

This is different from pure FollowPath playback:
  Old:
    waypoints.txt -> Path -> FollowPath

  New:
    waypoints.txt -> target waypoint -> ComputePathToPose -> FollowPath
"""

import os
import math
import time
import threading
from typing import List, Optional, Tuple, Any

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath, ComputePathToPose


class Waypoint:
    def __init__(self, x, y, yaw, qx, qy, qz, qw):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.qx = float(qx)
        self.qy = float(qy)
        self.qz = float(qz)
        self.qw = float(qw)


class WaypointReplanningNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_replanning_navigator')

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('spacing_m', 2.0)
        self.declare_parameter('waypoints_file', '/tmp/waypoints.txt')

        # Nav2 action names
        self.declare_parameter('compute_path_action', '/compute_path_to_pose')
        self.declare_parameter('follow_path_action', '/follow_path')

        # Must match planner_server planner_plugins
        # In your yaml:
        # planner_plugins: ["GridBased"]
        self.declare_parameter('planner_id', 'GridBased')

        # Must match controller_server controller_plugins
        # In your yaml:
        # controller_plugins: ["FollowPath"]
        self.declare_parameter('controller_id', 'FollowPath')

        # Must match controller_server current_goal_checker
        self.declare_parameter('goal_checker_id', 'goal_checker')

        # Playback behavior
        self.declare_parameter('start_from_nearest', True)
        self.declare_parameter('use_tangent_yaw_for_target', True)

        # How far ahead on the recorded route to choose the temporary target.
        # Larger = fewer planner calls, but less reactive.
        # Smaller = more frequent replanning.
        self.declare_parameter('target_lookahead_wps', 15)

        # If close to this distance from final waypoint, finish playback.
        self.declare_parameter('final_goal_tolerance_m', 2.5)

        # Timeouts
        self.declare_parameter('compute_path_timeout_sec', 5.0)
        self.declare_parameter('follow_path_timeout_sec', 25.0)

        # Failure handling
        self.declare_parameter('max_consecutive_failures', 10)

        # Seconds to wait after a FollowPath failure before replanning.
        # Gives the controller time to decelerate to a stop.
        self.declare_parameter('replan_delay_sec', 1.5)

        # Hard cap on how far ahead (in metres from current robot position) the
        # planning target may be. Must be less than half the global costmap
        # width/height, or planning will always fail with "goal off costmap".
        # Set to 0.0 to disable (rely on target_lookahead_wps alone).
        self.declare_parameter('max_plan_dist_m', 20.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.spacing_m = float(self.get_parameter('spacing_m').value)
        self.waypoints_file = self.get_parameter('waypoints_file').value

        self.compute_path_action = self.get_parameter('compute_path_action').value
        self.follow_path_action = self.get_parameter('follow_path_action').value

        self.planner_id = self.get_parameter('planner_id').value
        self.controller_id = self.get_parameter('controller_id').value
        self.goal_checker_id = self.get_parameter('goal_checker_id').value

        self.start_from_nearest = bool(self.get_parameter('start_from_nearest').value)
        self.use_tangent_yaw_for_target = bool(
            self.get_parameter('use_tangent_yaw_for_target').value
        )

        self.target_lookahead_wps = int(
            self.get_parameter('target_lookahead_wps').value
        )
        self.final_goal_tolerance_m = float(
            self.get_parameter('final_goal_tolerance_m').value
        )

        self.compute_path_timeout_sec = float(
            self.get_parameter('compute_path_timeout_sec').value
        )
        self.follow_path_timeout_sec = float(
            self.get_parameter('follow_path_timeout_sec').value
        )

        self.max_consecutive_failures = int(
            self.get_parameter('max_consecutive_failures').value
        )

        self.replan_delay_sec = float(
            self.get_parameter('replan_delay_sec').value
        )

        self.max_plan_dist_m = float(self.get_parameter('max_plan_dist_m').value)

        if self.target_lookahead_wps < 1:
            self.get_logger().warn(
                'target_lookahead_wps must be >= 1. Forcing to 1.'
            )
            self.target_lookahead_wps = 1

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
        # Action clients
        # ------------------------------------------------------------
        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            self.compute_path_action
        )

        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            self.follow_path_action
        )

        # ------------------------------------------------------------
        # Keyboard thread
        # ------------------------------------------------------------
        self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.kb_thread.start()

        self.get_logger().info(
            '\n'
            + '=' * 78 + '\n'
            + '  Waypoint Recorder + Replanning Nav2 Player\n'
            + '=' * 78 + '\n'
            + f'  State                    : RECORDING\n'
            + f'  Odom topic               : {self.odom_topic}\n'
            + f'  Spacing                  : {self.spacing_m} m\n'
            + f'  Waypoints file           : {self.waypoints_file}\n'
            + f'  ComputePathToPose action : {self.compute_path_action}\n'
            + f'  FollowPath action        : {self.follow_path_action}\n'
            + f'  Planner ID               : {self.planner_id}\n'
            + f'  Controller ID            : {self.controller_id}\n'
            + f'  Goal checker ID          : {self.goal_checker_id}\n'
            + f'  Target lookahead wps     : {self.target_lookahead_wps}\n'
            + f'  Final goal tolerance     : {self.final_goal_tolerance_m} m\n'
            + '=' * 78 + '\n'
            + '  Drive the vehicle to record waypoints.\n'
            + '  Press S + Enter to stop recording and save.\n'
            + '  Press P + Enter to play with replanning.\n'
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
                f.write('# columns: x y yaw qx qy qz qw\n')

                for wp in self.waypoints:
                    f.write(
                        f'{wp.x:.6f} {wp.y:.6f} {wp.yaw:.9f} '
                        f'{wp.qx:.9f} {wp.qy:.9f} {wp.qz:.9f} {wp.qw:.9f}\n'
                    )

            self.get_logger().info(
                f'[SAVE] Saved {len(self.waypoints)} waypoints to "{self.waypoints_file}".'
            )
            self.get_logger().info('Press P + Enter to start replanning playback.')

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

                    if len(parts) != 7:
                        self.get_logger().warn(
                            f'Skipping malformed waypoint line: {line}'
                        )
                        continue

                    x, y, yaw, qx, qy, qz, qw = map(float, parts)
                    loaded_wps.append(Waypoint(x, y, yaw, qx, qy, qz, qw))

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
            f'[NAV] Waiting for ComputePathToPose action server: {self.compute_path_action}'
        )
        if not self.compute_path_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f'ComputePathToPose action server "{self.compute_path_action}" not available.'
            )
            self.state = 'IDLE'
            return

        self.get_logger().info(
            f'[NAV] Waiting for FollowPath action server: {self.follow_path_action}'
        )
        if not self.follow_path_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f'FollowPath action server "{self.follow_path_action}" not available.'
            )
            self.state = 'IDLE'
            return

        consecutive_failures = 0
        # extra_lookahead is capped at one step to skip a single blocked waypoint
        # cluster; it resets on every loop so it doesn't compound across segments.
        extra_lookahead = 0

        while rclpy.ok() and not self.nav_cancel_requested:
            if self.latest_odom is None:
                self.get_logger().warn('[NAV] No odometry yet. Cannot navigate.')
                consecutive_failures += 1
                if consecutive_failures >= self.max_consecutive_failures:
                    break
                continue

            nearest_idx = self.find_nearest_waypoint_index(self.waypoints)
            if nearest_idx is None:
                consecutive_failures += 1
                if consecutive_failures >= self.max_consecutive_failures:
                    break
                continue

            final_dist = self.distance_from_odom_to_waypoint(self.waypoints[-1])
            if final_dist is not None and final_dist <= self.final_goal_tolerance_m:
                self.get_logger().info(
                    f'[NAV] Final waypoint reached. distance={final_dist:.2f} m'
                )
                break

            target_idx = min(
                nearest_idx + self.target_lookahead_wps + extra_lookahead,
                len(self.waypoints) - 1
            )

            # Cap target_idx so the planning goal stays within the global
            # costmap radius. Walk backward from target_idx until we find
            # a waypoint within max_plan_dist_m of the current robot pose.
            if self.max_plan_dist_m > 0.0 and self.latest_odom is not None:
                rx = self.latest_odom.pose.pose.position.x
                ry = self.latest_odom.pose.pose.position.y
                for i in range(target_idx, nearest_idx, -1):
                    wp = self.waypoints[i]
                    if math.hypot(wp.x - rx, wp.y - ry) <= self.max_plan_dist_m:
                        target_idx = i
                        break

            # If nearest is already at the end, finish.
            if target_idx <= nearest_idx and target_idx >= len(self.waypoints) - 1:
                self.get_logger().info('[NAV] Reached end of waypoint list.')
                break

            target_pose = self.make_target_pose(target_idx)

            self.get_logger().info(
                f'[NAV] Replanning from current odom to waypoint index={target_idx} '
                f'(nearest={nearest_idx})'
            )

            planned_path = self.compute_path_to_pose(target_pose)

            if planned_path is None or len(planned_path.poses) < 2:
                self.get_logger().warn(
                    f'[NAV] Planning failed to target index={target_idx}.'
                )
                consecutive_failures += 1

                if consecutive_failures >= self.max_consecutive_failures:
                    self.get_logger().error(
                        '[NAV] Too many consecutive failures. Stopping playback.'
                    )
                    break

                continue

            # Compute timeout from actual planned path length so long segments
            # (after corner turns or obstacle detours) don't spuriously time out.
            # Use min expected speed (regulated_linear_scaling_min_speed) plus a buffer.
            path_length = self.compute_path_length(planned_path)
            dynamic_timeout = max(path_length / 0.3 + 15.0, self.follow_path_timeout_sec)

            self.get_logger().info(
                f'[NAV] Planner returned path with {len(planned_path.poses)} poses '
                f'({path_length:.1f} m). Follow timeout={dynamic_timeout:.1f} s.'
            )

            ok = self.follow_planned_path(planned_path, timeout_sec=dynamic_timeout)

            if ok:
                consecutive_failures = 0
                extra_lookahead = 0
                self.get_logger().info(
                    '[NAV] FollowPath segment succeeded. Will replan next segment.'
                )
            else:
                consecutive_failures += 1
                # Bump lookahead by one step to aim past the blocked cluster,
                # but cap it so we don't plan an unboundedly long path.
                extra_lookahead = min(
                    extra_lookahead + self.target_lookahead_wps,
                    self.target_lookahead_wps
                )
                self.get_logger().warn(
                    f'[NAV] FollowPath failed or timed out. '
                    f'Replanning attempt count={consecutive_failures}/'
                    f'{self.max_consecutive_failures}, '
                    f'extra_lookahead={extra_lookahead}'
                )

                if consecutive_failures >= self.max_consecutive_failures:
                    self.get_logger().error(
                        '[NAV] Too many consecutive FollowPath failures. Stopping.'
                    )
                    break

                # Pause before replanning so the controller can decelerate
                # to a stop. Without this, a rapid-fire replan loop keeps the
                # controller active and the robot never actually slows down.
                self.get_logger().info(
                    f'[NAV] Waiting {self.replan_delay_sec}s before replanning...'
                )
                time.sleep(self.replan_delay_sec)
                # Reset extra_lookahead: if the robot moved during the pause,
                # the new nearest_idx already reflects the progress made.
                extra_lookahead = 0

        # Cancel any still-active controller goal so the robot actually stops.
        handle = self._active_follow_goal_handle
        if handle is not None:
            self.get_logger().info('[NAV] Cancelling active FollowPath goal to stop robot.')
            try:
                handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'[NAV] Cancel on exit failed: {e}')
            self._active_follow_goal_handle = None

        self.get_logger().info('[NAV] Replanning playback finished. State -> IDLE.')
        self.state = 'IDLE'

    # ------------------------------------------------------------
    # Planner action
    # ------------------------------------------------------------
    def compute_path_to_pose(self, target_pose: PoseStamped) -> Optional[Path]:
        start_pose = self.make_current_pose_stamped()
        if start_pose is None:
            self.get_logger().warn('[PLAN] Cannot create start pose from odom.')
            return None

        goal = ComputePathToPose.Goal()
        goal.start = start_pose
        goal.goal = target_pose
        goal.planner_id = self.planner_id
        goal.use_start = True

        self.get_logger().info(
            f'[PLAN] Sending ComputePathToPose. '
            f'planner_id="{self.planner_id}", '
            f'frame="{target_pose.header.frame_id}", '
            f'target=({target_pose.pose.position.x:.2f}, '
            f'{target_pose.pose.position.y:.2f})'
        )

        send_future = self.compute_path_client.send_goal_async(goal)

        ok, goal_handle = self.wait_for_future_no_spin(
            future=send_future,
            timeout_sec=self.compute_path_timeout_sec,
            description='send ComputePathToPose goal'
        )

        if not ok or goal_handle is None:
            return None

        if not goal_handle.accepted:
            self.get_logger().warn('[PLAN] ComputePathToPose goal rejected.')
            return None

        result_future = goal_handle.get_result_async()

        ok, result_msg = self.wait_for_future_no_spin(
            future=result_future,
            timeout_sec=self.compute_path_timeout_sec,
            description='ComputePathToPose result'
        )

        if not ok or result_msg is None:
            self.get_logger().warn('[PLAN] ComputePathToPose result timeout/failure.')
            return None

        # action_msgs/GoalStatus:
        # STATUS_SUCCEEDED = 4
        if result_msg.status != 4:
            self.get_logger().warn(
                f'[PLAN] ComputePathToPose did not succeed. status={result_msg.status}'
            )
            return None

        path = result_msg.result.path

        if path.header.frame_id == '':
            path.header.frame_id = self.frame_id

        return path

    # ------------------------------------------------------------
    # Controller action
    # ------------------------------------------------------------
    def follow_planned_path(self, path_msg: Path, timeout_sec: Optional[float] = None) -> bool:
        if timeout_sec is None:
            timeout_sec = self.follow_path_timeout_sec

        goal = FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id

        self.get_logger().info(
            f'[CTRL] Sending FollowPath with {len(path_msg.poses)} poses. '
            f'controller_id="{self.controller_id}"'
        )

        send_future = self.follow_path_client.send_goal_async(goal)

        ok, goal_handle = self.wait_for_future_no_spin(
            future=send_future,
            timeout_sec=5.0,
            description='send FollowPath goal'
        )

        if not ok or goal_handle is None:
            return False

        if not goal_handle.accepted:
            self.get_logger().warn('[CTRL] FollowPath goal rejected.')
            return False

        self._active_follow_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()

        ok, result_msg = self.wait_for_future_no_spin(
            future=result_future,
            timeout_sec=timeout_sec,
            description='FollowPath result'
        )

        if not ok:
            self.get_logger().warn(
                '[CTRL] FollowPath timed out. Cancelling current controller goal.'
            )
            try:
                goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f'[CTRL] Cancel failed: {e}')
            return False

        self._active_follow_goal_handle = None

        if result_msg is None:
            return False

        if result_msg.status != 4:
            self.get_logger().warn(
                f'[CTRL] FollowPath did not succeed. status={result_msg.status}'
            )
            return False

        return True

    @staticmethod
    def compute_path_length(path: Path) -> float:
        poses = path.poses
        length = 0.0
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
            dy = poses[i].pose.position.y - poses[i - 1].pose.position.y
            length += math.hypot(dx, dy)
        return length

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
            qx, qy, qz, qw = self.quat_from_yaw(yaw)
        else:
            qx, qy, qz, qw = wp.qx, wp.qy, wp.qz, wp.qw

        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def distance_from_odom_to_waypoint(self, wp: Waypoint) -> Optional[float]:
        if self.latest_odom is None:
            return None

        rx = self.latest_odom.pose.pose.position.x
        ry = self.latest_odom.pose.pose.position.y

        return math.hypot(wp.x - rx, wp.y - ry)

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


