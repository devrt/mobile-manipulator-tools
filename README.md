mobile_manipulator_tools
=========================

ROS packages to enable mobile-manipulator planning with MoveIt!


add_odom_virtual_joints
-----------------------
Add odom_x, odom_y, odom_r virtual joints (linear, linear, revolution) to your existing URDF model to let any kinematics library to calculate inverse kinematics of the mobile base.


loopback_robot_hardware
-----------------------
Create virtual joint that can be accessed from ros_controllers and loopback joint value from the other joint.
This virtual joint can be used as a shared memory between the realtime controllers.

We use this module to create odom_x, odom_y, odom_r virtual joints which accept joint trajectory command from MoveIt! and subsequently used by mobile base controller.


cmd_vel_controller
-------------------
This ros_controller will subscribe to `/cmd_vel` topic and write velocity command to odom_x, odom_y, odom_r virtual joints.

We use this controller to accept command from ros_nav.


Author
------
Yosuke Matsusaka (MID Academic Promotions, Inc.)