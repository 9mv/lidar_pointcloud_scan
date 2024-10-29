cmake_minimum_required(VERSION 3.8)
project(lidar_pointcloud_scan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Angle.msg"
)

set(srv_files
  "srv/StartScan.srv"  
  "srv/StopScan.srv"
)

set(action_files
  "action/MotorScan.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(lidar_rotation_motor src/lidar_rotation_motor.cpp
  include/I2C/I2C.cpp
  include/PCA9685/PCA9685.cpp
)
ament_target_dependencies(lidar_rotation_motor 
  rclcpp
  rclcpp_action
  action_msgs
  )

add_executable(point_cloud_transformer src/point_cloud_transformer.cpp)
ament_target_dependencies(point_cloud_transformer 
  rclcpp
  sensor_msgs
  )

add_executable(device_orchestrator src/device_orchestrator.cpp)
ament_target_dependencies(device_orchestrator 
  rclcpp
  rclcpp_action
  action_msgs
  sensor_msgs
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
    lidar_rotation_motor
    point_cloud_transformer
    device_orchestrator
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(include)

target_include_directories(lidar_rotation_motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(lidar_rotation_motor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(point_cloud_transformer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(point_cloud_transformer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(device_orchestrator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(device_orchestrator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(lidar_rotation_motor "${cpp_typesupport_target}")
target_link_libraries(point_cloud_transformer "${cpp_typesupport_target}")
target_link_libraries(device_orchestrator "${cpp_typesupport_target}")

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
