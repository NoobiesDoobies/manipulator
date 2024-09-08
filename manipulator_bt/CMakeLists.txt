cmake_minimum_required(VERSION 3.8)
project(manipulator_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(manipulator_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(cv_bridge REQUIRED)


include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(pickup_box_bt src/pickup_box_bt.cpp)
ament_target_dependencies(pickup_box_bt
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "unique_identifier_msgs"
  "builtin_interfaces"
  "geometry_msgs"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "action_msgs"
  "manipulator_msgs"
  "behaviortree_ros2"
)



install(TARGETS
  pickup_box_bt
  DESTINATION lib/${PROJECT_NAME})


  add_executable(manipulator_client src/manipulator_client.cpp)
  ament_target_dependencies(manipulator_client
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components"
    "unique_identifier_msgs"
    "builtin_interfaces"
    "geometry_msgs"
    "std_msgs"
    "geometry_msgs"
    "sensor_msgs"
    "action_msgs"
    "manipulator_msgs"
  )
  
  
  
  install(TARGETS
  manipulator_client
    DESTINATION lib/${PROJECT_NAME})
  


add_executable(detect_aruco src/detect_aruco.cpp)
ament_target_dependencies(detect_aruco
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "unique_identifier_msgs"
  "builtin_interfaces"
  "geometry_msgs"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "action_msgs"
  "manipulator_msgs"
  "OpenCV"
  "cv_bridge"
)

install(TARGETS
  detect_aruco
  DESTINATION lib/${PROJECT_NAME})

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

install(
  DIRECTORY  launch  behaviour_trees
  DESTINATION share/${PROJECT_NAME}
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "action/ManipulatorTask.action"
#   DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs unique_identifier_msgs action_msgs
# )

ament_package()
