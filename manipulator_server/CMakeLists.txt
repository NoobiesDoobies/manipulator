cmake_minimum_required(VERSION 3.8)
project(manipulator_server)

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
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(manipulator_msgs REQUIRED)

add_library(manipulator_task_server SHARED src/manipulator_task_server.cpp)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(manipulator_task_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

  

target_compile_definitions(manipulator_task_server
    PRIVATE "MANIPULATOR_TASK_SERVER_CPP_BUILDING_DLL")
  ament_target_dependencies(manipulator_task_server
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components"
    "moveit_ros_planning_interface"
    "unique_identifier_msgs"
    "builtin_interfaces"
    "geometry_msgs"
    "std_msgs"
    "sensor_msgs"
    "action_msgs"
    "manipulator_msgs"
    
    )
  rclcpp_components_register_node(manipulator_task_server 
    PLUGIN "manipulator_server_remote::TaskServer" 
    EXECUTABLE manipulator_task_server_node
)

install(TARGETS 
  manipulator_task_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

add_library(mobile_bot_task_server SHARED src/mobile_bot_task_server.cpp)

target_include_directories(mobile_bot_task_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

target_compile_definitions(mobile_bot_task_server
    PRIVATE "MOBILE_BOT_TASK_SERVER_CPP_BUILDING_DLL")
  ament_target_dependencies(mobile_bot_task_server
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components"
    "moveit_ros_planning_interface"
    "unique_identifier_msgs"
    "builtin_interfaces"
    "geometry_msgs"
    "std_msgs"
    "sensor_msgs"
    "geometry_msgs"
    "manipulator_msgs"
)

rclcpp_components_register_node(mobile_bot_task_server 
  PLUGIN "mobile_bot_server_remote::TaskServer" 
  EXECUTABLE mobile_bot_task_server_node
)

install(TARGETS 
  mobile_bot_task_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
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

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "action/ManipulatorTask.action"
#   DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs unique_identifier_msgs action_msgs
# )

ament_package()
