cmake_minimum_required(VERSION 3.8)
project(Nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch archive
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  src/record_followpath.py
  src/record_poses_and_publish_as_goals.py
  src/waypoint_replanning_navigator.py
  src/waypoints.txt
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
