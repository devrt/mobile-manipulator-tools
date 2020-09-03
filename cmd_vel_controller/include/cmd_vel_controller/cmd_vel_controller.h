/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, MID Academic Promotions, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the MID Academic Promotions nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Yosuke Matsusaka
 */

#pragma once

#include <string>
#include <vector>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/Twist.h>

namespace cmd_vel_controller
{

    class CmdVelController
        : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
    {
    public:
        CmdVelController();

        bool init(hardware_interface::VelocityJointInterface *hw,
                  ros::NodeHandle &root_nh,
                  ros::NodeHandle &controller_nh);

        void update(const ros::Time &time, const ros::Duration &period);

        void starting(const ros::Time &time);

        void stopping(const ros::Time &time);

    private:
        std::string name_;

        // Joint handles:
        hardware_interface::JointHandle odom_x_;
        hardware_interface::JointHandle odom_y_;
        hardware_interface::JointHandle odom_r_;

        // Velocity command subscribing:
        struct Command
        {
            double lin_x;
            double lin_y;
            double ang;
            ros::Time stamp;

            Command() : lin_x(0.0), lin_y(0.0), ang(0.0), stamp(0.0) {}
        };
        realtime_tools::RealtimeBuffer<Command> command_;
        Command command_struct_;
        ros::Subscriber sub_command_;

        double cmd_vel_timeout_;

    protected:
        void cmdVelCallback(const geometry_msgs::Twist &command);
    };

    PLUGINLIB_EXPORT_CLASS(cmd_vel_controller::CmdVelController, controller_interface::ControllerBase);
} // namespace cmd_vel_controller