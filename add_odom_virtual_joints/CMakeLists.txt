cmake_minimum_required(VERSION 2.8.6 FATAL_ERROR)
project(add_odom_virtual_joints)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  DEPENDS
)

catkin_install_python(PROGRAMS add_odom_virtual_joints.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
