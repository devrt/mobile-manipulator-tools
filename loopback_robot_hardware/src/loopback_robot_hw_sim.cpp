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
 *   * Neither the name of the MID Academic Promotions
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
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

/* Author: Yosuke Matsusaka
*/

#include <loopback_robot_hardware/loopback_robot_hw_sim.h>

namespace loopback_robot_hardware
{

    bool LoopbackRobotHWSim::initSim(
        const std::string &robot_namespace,
        ros::NodeHandle model_nh,
        gazebo::physics::ModelPtr parent_model,
        const urdf::Model *const urdf_model,
        std::vector<transmission_interface::TransmissionInfo> transmissions)
    {
        model_nh.getParam("loopback_position_interfaces", loopback_position_interfaces_);
        model_nh.getParam("loopback_velocity_interfaces", loopback_velocity_interfaces_);
        model_nh.getParam("loopback_effort_interfaces", loopback_effort_interfaces_);

        // create default hardware if none defined
        if (loopback_position_interfaces_.size() == 0 &&
            loopback_velocity_interfaces_.size() == 0 &&
            loopback_velocity_interfaces_.size() == 0)
        {
            loopback_velocity_interfaces_.push_back("odom_x");
            loopback_velocity_interfaces_.push_back("odom_y");
            loopback_velocity_interfaces_.push_back("odom_r");
        }

        gazebo_ros_control::DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);

        loopback_joint_names_.resize(0);
        loopback_joint_control_methods_.resize(0);
        for (auto n : loopback_position_interfaces_)
        {
            loopback_joint_names_.push_back(n);
            loopback_joint_control_methods_.push_back(POSITION);
        }
        for (auto n : loopback_velocity_interfaces_)
        {
            loopback_joint_names_.push_back(n);
            loopback_joint_control_methods_.push_back(VELOCITY);
        }
        for (auto n : loopback_effort_interfaces_)
        {
            loopback_joint_names_.push_back(n);
            loopback_joint_control_methods_.push_back(EFFORT);
        }

        loopback_n_dof_ = loopback_joint_names_.size();
        loopback_joint_position_.resize(loopback_n_dof_);
        loopback_joint_velocity_.resize(loopback_n_dof_);
        loopback_joint_effort_.resize(loopback_n_dof_);
        loopback_joint_position_desired_.resize(loopback_n_dof_);
        loopback_joint_velocity_desired_.resize(loopback_n_dof_);
        loopback_joint_effort_desired_.resize(loopback_n_dof_);
        loopback_joint_command_.resize(loopback_n_dof_);
        loopback_joint_command_prev_.resize(loopback_n_dof_);
        loopback_joint_command_desired_.resize(loopback_n_dof_);

        for (unsigned int j = 0; j < loopback_n_dof_; j++)
        {
            loopback_joint_position_[j] = 0.0;
            loopback_joint_velocity_[j] = 0.0;
            loopback_joint_effort_[j] = 0.0;
            loopback_joint_position_desired_[j] = 0.0;
            loopback_joint_velocity_desired_[j] = 0.0;
            loopback_joint_effort_desired_[j] = 0.0;
            loopback_joint_command_[j] = 0.0;
            loopback_joint_command_prev_[j] = 0.0;
            loopback_joint_command_desired_[j] = 0.0;

            std::string lp_name(loopback_joint_names_[j] + "_loopback");

            // original joint will receive desired joint command from the other controller
            js_interface_.registerHandle(hardware_interface::JointStateHandle(
                loopback_joint_names_[j], &loopback_joint_position_[j], &loopback_joint_velocity_[j], &loopback_joint_effort_[j]));
            hardware_interface::JointHandle joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(loopback_joint_names_[j]),
                                                                                           &loopback_joint_command_desired_[j]);

            // joint with name *_loopback will accept current state of the joint as a command (and relay the state to the other controller)
            js_interface_.registerHandle(hardware_interface::JointStateHandle(
                lp_name, &loopback_joint_position_desired_[j], &loopback_joint_velocity_desired_[j], &loopback_joint_effort_desired_[j]));
            hardware_interface::JointHandle joint_handle_lp = hardware_interface::JointHandle(js_interface_.getHandle(lp_name),
                                                                                              &loopback_joint_command_[j]);
            switch (loopback_joint_control_methods_[j])
            {
            case POSITION:
                pj_interface_.registerHandle(joint_handle);
                pj_interface_.registerHandle(joint_handle_lp);
                break;
            case VELOCITY:
                vj_interface_.registerHandle(joint_handle);
                vj_interface_.registerHandle(joint_handle_lp);
                break;
            case EFFORT:
                ej_interface_.registerHandle(joint_handle);
                ej_interface_.registerHandle(joint_handle_lp);
                break;
            }
        }

        // Re-register interfaces
        registerInterface(&js_interface_);
        registerInterface(&pj_interface_);
        registerInterface(&vj_interface_);
        registerInterface(&ej_interface_);

        prev_time_ = 0.0;

        return true;
    }

    void LoopbackRobotHWSim::readSim(ros::Time time, ros::Duration period)
    {
        gazebo_ros_control::DefaultRobotHWSim::readSim(time, period);
        double t = time.toSec();
        double dt;

        if (prev_time_ == 0.0) {
            dt = period.toSec();
        } else {
            dt = t - prev_time_;
        }

        if (dt == 0.0) return;

        for (unsigned int j = 0; j < loopback_n_dof_; j++)
        {
            double cmd = loopback_joint_command_[j];
            double prevcmd = loopback_joint_command_prev_[j];
            switch (loopback_joint_control_methods_[j])
            {
            case POSITION:
            {
                double prevvel = loopback_joint_velocity_[j];
                loopback_joint_position_[j] = cmd;
                loopback_joint_velocity_[j] = (cmd - prevcmd) / dt;                       // differentiate
                loopback_joint_effort_[j] = (loopback_joint_velocity_[j] - prevvel) / dt; // differentiate
                loopback_joint_position_desired_[j] = loopback_joint_command_desired_[j];
            }
            break;
            case VELOCITY:
            {
                loopback_joint_position_[j] += cmd * dt;          // integrate
                loopback_joint_velocity_[j] = cmd;
                loopback_joint_effort_[j] = (cmd - prevcmd) / dt; // differentiate
                loopback_joint_velocity_desired_[j] = loopback_joint_command_desired_[j];
            }
            break;
            case EFFORT:
            {
                loopback_joint_effort_[j] = cmd;
                loopback_joint_velocity_[j] += cmd * dt;                         // integrate
                loopback_joint_position_[j] += loopback_joint_velocity_[j] * dt; // integrate
                loopback_joint_effort_desired_[j] = loopback_joint_command_desired_[j];
            }
            break;
            }
            loopback_joint_command_prev_[j] = cmd;
        }

        prev_time_ = t;
    }

    void LoopbackRobotHWSim::writeSim(ros::Time time, ros::Duration period)
    {
        gazebo_ros_control::DefaultRobotHWSim::writeSim(time, period);
    }

} // namespace loopback_robot_hardware

PLUGINLIB_EXPORT_CLASS(loopback_robot_hardware::LoopbackRobotHWSim, gazebo_ros_control::RobotHWSim)