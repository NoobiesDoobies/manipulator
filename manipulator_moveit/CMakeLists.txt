cmake_minimum_required(VERSION 3.8)
project(manipulator_moveit)

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



install(
  DIRECTORY launch config rviz  
  DESTINATION share/${PROJECT_NAME}
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "action/ManipulatorTask.action"
#   DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs unique_identifier_msgs action_msgs
# )

ament_package()
