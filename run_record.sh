#!/usr/bin/env bash
# run_record.sh
# Runs the waypoint recorder/navigator interactively inside the nav2 container.
# Saves the waypoints to the mounted config folder so they persist on the host.

docker exec -it nav2 bash -c "source /entrypoint.sh && ros2 run Nav2 waypoint_replanning_navigator.py --ros-args \
  -p odom_topic:=/Odometry \
  -p spacing_m:=1.0 \
  -p waypoints_file:=/workspace/ros_ws/install/Nav2/share/Nav2/config/waypoints.txt \
  -p compute_path_action:=/compute_path_to_pose \
  -p follow_path_action:=/follow_path \
  -p planner_id:=GridBased \
  -p controller_id:=FollowPath \
  -p goal_checker_id:=goal_checker \
  -p target_lookahead_wps:=15 \
  -p follow_path_timeout_sec:=25.0"
