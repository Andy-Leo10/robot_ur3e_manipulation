cmake_minimum_required(VERSION 3.8)
project(robot_ur3e_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(grasping_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Define a function to set target dependencies
function(set_my_target_deps target)
  ament_target_dependencies(${target}
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    interactive_markers
    geometric_shapes
    control_msgs
    moveit_msgs
    tf2
    tf2_geometry_msgs
    grasping_msgs
	std_msgs
  )
endfunction()

# generate executable for pick_and_place_primitive
add_executable(pick_and_place_primitive src/pick_and_place_primitive.cpp)
ament_target_dependencies(pick_and_place_primitive
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  interactive_markers
  geometric_shapes
  control_msgs
  moveit_msgs
  tf2
  tf2_geometry_msgs
  grasping_msgs
)

# generate executable for pick_and_place_advanced
add_executable(pick_and_place_advanced src/pick_and_place_advanced.cpp)
# ament_target_dependencies(pick_and_place_advanced
#   rclcpp
#   rclcpp_action
#   moveit_core
#   moveit_ros_planning
#   moveit_ros_planning_interface
#   interactive_markers
#   geometric_shapes
#   control_msgs
#   moveit_msgs
#   tf2
#   tf2_geometry_msgs
#   grasping_msgs
# )
# Use the function to set dependencies for your target
set_my_target_deps(pick_and_place_advanced)

add_executable(scene_objects src/scene_objects.cpp)
set_my_target_deps(scene_objects)

# install executables
install(TARGETS
  pick_and_place_primitive
  pick_and_place_advanced
  scene_objects
  DESTINATION lib/${PROJECT_NAME}
)

# install directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
