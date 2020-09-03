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

#ifndef _LOOPBACK_ROBOT_HW_SIM_H_
#define _LOOPBACK_ROBOT_HW_SIM_H_

#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <gazebo_ros_control/robot_hw_sim.h>

namespace loopback_robot_hardware
{

    class LoopbackRobotHWSim : public gazebo_ros_control::DefaultRobotHWSim
    {
    public:
        virtual bool initSim(
            const std::string &robot_namespace,
            ros::NodeHandle model_nh,
            gazebo::physics::ModelPtr parent_model,
            const urdf::Model *const urdf_model,
            std::vector<transmission_interface::TransmissionInfo> transmissions);

        virtual void readSim(ros::Time time, ros::Duration period);

        virtual void writeSim(ros::Time time, ros::Duration period);

    protected:
        std::vector<std::string> loopback_position_interfaces_;
        std::vector<std::string> loopback_velocity_interfaces_;
        std::vector<std::string> loopback_effort_interfaces_;

        unsigned int loopback_n_dof_;
        std::vector<std::string> loopback_joint_names_;
        std::vector<ControlMethod> loopback_joint_control_methods_;
        std::vector<double> loopback_joint_position_;
        std::vector<double> loopback_joint_velocity_;
        std::vector<double> loopback_joint_effort_;
        std::vector<double> loopback_joint_position_desired_;
        std::vector<double> loopback_joint_velocity_desired_;
        std::vector<double> loopback_joint_effort_desired_;
        std::vector<double> loopback_joint_command_;
        std::vector<double> loopback_joint_command_prev_;
        std::vector<double> loopback_joint_command_desired_;

        double prev_time_;
    };

    typedef boost::shared_ptr<LoopbackRobotHWSim> LoopbackRobotHWSimPtr;

} // namespace loopback_robot_hardware

#endif // #ifndef _LOOPBACK_ROBOT_HW_SIM_H_
