#!/usr/bin/env python

##
# Software License Agreement (BSD License)
#
#  Copyright (c) 2020, MID Academic Promotions, Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the MID Academic Promotions nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
##

# Author: Yosuke Matsusaka

import rospy
from threading import Lock
from control_msgs.msg import FollowJointTrajectoryActionResult
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

class ControllerSwitcher:
    last_cmd_time = 0
    lock = None
    pub = None
    sub_cmd_vel = None
    sub_trajectory_status = None
    switch_controller = None
    CMDVEL_MODE = 1
    TRAJ_MODE = 2
    current_mode = 0
    monitor_topic_name = 'cmd_vel'
    publish_topic_name = 'cmd_vel_controller/cmd_vel'
    cmd_vel_timeout = 0.5
    cmdvel_controller_name = 'cmd_vel_controller'
    traj_controller_name = 'omni_trajectory_controller'

    def __init__(self):
        self.lock = Lock()

    def switch_to_cmdvel(self):
        if self.current_mode != self.CMDVEL_MODE:
            rospy.loginfo("switch from %s to %s", self.traj_controller_name, self.cmdvel_controller_name)
            with self.lock:
                self.current_mode = self.CMDVEL_MODE
                ret = self.switch_controller([self.cmdvel_controller_name], [self.traj_controller_name], SwitchControllerRequest.BEST_EFFORT, True, 0)
            return ret
        return True

    def switch_to_traj(self):
        if self.current_mode != self.TRAJ_MODE:
            rospy.loginfo("switch from %s to %s", self.cmdvel_controller_name, self.traj_controller_name)
            with self.lock:
                self.current_mode = self.TRAJ_MODE
                ret = self.switch_controller([self.traj_controller_name], [self.cmdvel_controller_name], SwitchControllerRequest.BEST_EFFORT, True, 0)
            return ret
        return True

    def cmd_vel_callback(self, data):
        self.last_cmd_time = rospy.get_time()
        self.switch_to_cmdvel()
        self.pub.publish(data)

    def omni_trajectory_status_callback(self, data):
        if data.status.status == GoalStatus.ABORTED:
            rospy.loginfo("%s aborted", self.traj_controller_name)
            self.last_cmd_time = rospy.get_time()
            self.switch_to_cmdvel()
            hold_data = Twist() # data with zero velocity (=hold)
            for i in range(0, 10):
                self.last_cmd_time = rospy.get_time()
                self.pub.publish(hold_data)
                rospy.sleep(0.1)

    def start(self):
        rospy.init_node('controller_switcher', anonymous=True)
        
        self.monitor_topic_name = rospy.get_param('~cmd_vel_topic_name', self.monitor_topic_name)
        self.publish_topic_name = rospy.get_param('~publish_topic_name', self.publish_topic_name)
        self.cmd_vel_timeout = rospy.get_param('~cmd_vel_timeout', self.cmd_vel_timeout)
        self.cmdvel_controller_name = rospy.get_param('~cmd_vel_controller_name', self.cmdvel_controller_name)
        self.traj_controller_name = rospy.get_param('~omni_trajectory_controller_name', self.traj_controller_name)
        controller_manager_name = rospy.get_param('~controller_manager_name', 'controller_manager')

        switch_controller_name = controller_manager_name + '/switch_controller'
        rospy.wait_for_service(switch_controller_name)
        self.switch_controller = rospy.ServiceProxy(switch_controller_name, SwitchController)

        self.switch_to_traj()

        self.last_cmd_time = rospy.get_time()
        self.pub = rospy.Publisher(self.publish_topic_name, Twist, queue_size=10)
        self.sub_cmd_vel = rospy.Subscriber(self.monitor_topic_name, Twist, self.cmd_vel_callback)
        self.sub_trajectory_status = rospy.Subscriber(self.traj_controller_name + '/follow_joint_trajectory/result', FollowJointTrajectoryActionResult, self.omni_trajectory_status_callback)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if rospy.get_time() - self.last_cmd_time > self.cmd_vel_timeout:
                self.switch_to_traj()
            r.sleep()

if __name__ == '__main__':
    try:
        main = ControllerSwitcher()
        main.start()
    except rospy.ROSInterruptException:
        pass