cmake_minimum_required(VERSION 3.5.1)
project(loopback_robot_hardware)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  std_msgs
  control_toolbox
  controller_manager
  hardware_interface
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  angles
  gazebo_ros_control
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_manager
    control_toolbox
    pluginlib
    hardware_interface
    transmission_interface
    joint_limits_interface
    urdf
    angles
    gazebo_ros_control
  INCLUDE_DIRS include
  LIBRARIES loopback_robot_hw_sim loopback_joint_state_controller
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Libraries
add_library(loopback_robot_hw_sim src/loopback_robot_hw_sim.cpp)
target_link_libraries(loopback_robot_hw_sim ${catkin_LIBRARIES})

add_library(loopback_joint_state_controller src/joint_state_controller.cpp)
target_link_libraries(loopback_joint_state_controller ${catkin_LIBRARIES})

## Install
install(TARGETS loopback_robot_hw_sim loopback_joint_state_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES robot_hw_sim_plugins.xml joint_state_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
