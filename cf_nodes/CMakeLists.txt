cmake_minimum_required(VERSION 3.8)
project(cf_nodes)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motion_capture_tracking_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(/usr/include/eigen3)
include_directories(
        include
        ${tf2_INCLUDE_DIRS}
        ${tf2_ros_INCLUDE_DIRS}
)


# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install launch and config files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(follow_platform
  cf_nodes/follow_platform.cpp
)
ament_target_dependencies(follow_platform
rclcpp
geometry_msgs
motion_capture_tracking_interfaces
crazyflie_interfaces
Eigen3
)

add_executable(safety
  cf_nodes/safety.cpp
)
ament_target_dependencies(safety
rclcpp
geometry_msgs
motion_capture_tracking_interfaces
crazyflie_interfaces
Eigen3
)

add_executable(teleop_fr
  cf_nodes/teleop_frysky_taranis.cpp
)
ament_target_dependencies(teleop_fr
rclcpp
geometry_msgs
motion_capture_tracking_interfaces
crazyflie_interfaces
Eigen3
tf2
tf2_ros
SDL2
)


# Install launch and config files. 
install(DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME}/ 
) 
 
install(TARGETS 
  follow_platform 
  DESTINATION lib/${PROJECT_NAME}) 
install(TARGETS 
  safety 
  DESTINATION lib/${PROJECT_NAME}) 
install(TARGETS 
  teleop_fr 
  DESTINATION lib/${PROJECT_NAME}) 
 
ament_export_dependencies( 
        tf2 
        tf2_ros 
)



ament_package()
