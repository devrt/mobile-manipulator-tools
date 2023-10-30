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

#include <cmath>
#include <cmd_vel_controller/cmd_vel_controller.h>

namespace cmd_vel_controller
{
    CmdVelController::CmdVelController()
        : command_struct_(), cmd_vel_timeout_(0.5)
    {
    }

    bool CmdVelController::init(hardware_interface::VelocityJointInterface *hw,
                                         ros::NodeHandle &root_nh,
                                         ros::NodeHandle &controller_nh)
    {
        const std::string complete_ns = controller_nh.getNamespace();
        std::size_t id = complete_ns.find_last_of("/");
        name_ = complete_ns.substr(id + 1);

        // Read parameters
        controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
        ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                                         << cmd_vel_timeout_ << "s.");

        odom_x_ = hw->getHandle("odom_x");
        odom_y_ = hw->getHandle("odom_y");
        odom_t_ = hw->getHandle("odom_t");

        // Subscribe to cmd_vel topic
        sub_command_ = controller_nh.subscribe("cmd_vel", 1, &CmdVelController::cmdVelCallback, this);

        return true;
    }

    void CmdVelController::update(const ros::Time &time, const ros::Duration &period)
    {
        double dt = period.toSec();

        // Read velocity command from non-realtime process
        Command curr_cmd = *(command_.readFromRT());
        const double cmd_dt = (time - curr_cmd.stamp).toSec();
        if (cmd_dt > cmd_vel_timeout_)
        {
            curr_cmd.lin_x = 0.0;
            curr_cmd.lin_y = 0.0;
            curr_cmd.ang = 0.0;
        }

        double ang = odom_t_.getPosition() + 0.5 * curr_cmd.ang * dt; // use Runge-Kutta 2nd
        double cosr = cos(ang);
        double sinr = sin(ang);
        double abs_dot_x = curr_cmd.lin_x * cosr - curr_cmd.lin_y * sinr;
        double abs_dot_y = curr_cmd.lin_x * sinr + curr_cmd.lin_y * cosr;

        odom_x_.setCommand(abs_dot_x);
        odom_y_.setCommand(abs_dot_y);
        odom_t_.setCommand(curr_cmd.ang);
    }

    void CmdVelController::starting(const ros::Time &time)
    {
    }

    void CmdVelController::stopping(const ros::Time &time)
    {
    }

    void CmdVelController::cmdVelCallback(const geometry_msgs::Twist &command)
    {
        if (isRunning())
        {
            // Send command from non-realtime callback process to realtime control process
            command_struct_.ang = command.angular.z;
            command_struct_.lin_x = command.linear.x;
            command_struct_.lin_y = command.linear.y;
            command_struct_.stamp = ros::Time::now();
            command_.writeFromNonRT(command_struct_);
        }
    }
} // namespace cmd_vel_controller