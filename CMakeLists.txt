cmake_minimum_required(VERSION 3.8)
project(mobile_bot)

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

add_library(task_server SHARED src/task_server.cpp)

target_include_directories(task_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  )
  

target_compile_definitions(task_server
    PRIVATE "TASK_SERVER_CPP_BUILDING_DLL")
  ament_target_dependencies(task_server
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
    )
  rclcpp_components_register_node(task_server 
    PLUGIN "mobile_bot::TaskServer" 
    EXECUTABLE task_server_node
)

install(TARGETS 
  task_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
  DIRECTORY description launch config rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msgs/action/ManipulatorTask.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs unique_identifier_msgs action_msgs
)

ament_package()
