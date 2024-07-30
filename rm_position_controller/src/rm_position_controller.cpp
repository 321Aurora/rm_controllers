/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include "rm_position_controller/rm_position_controller.h"
#include <rm_common/ros_utilities.h>

namespace rm_position_controller
{
bool RMPositionController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  ros::NodeHandle nh_pos1 = ros::NodeHandle(controller_nh, "pos_controller1");
  ros::NodeHandle nh_pos2 = ros::NodeHandle(controller_nh, "pos_controller2");
  threshold_ = getParam(controller_nh, "threshold", 0.);
  cmd_subscriber_ = controller_nh.subscribe<std_msgs::Float64>("/controllers/image_transmission_controller/command", 1,
                                                               &RMPositionController::commandCB, this);
  return (pos_ctrl1_.init(effort_joint_interface, nh_pos1) && pos_ctrl2_.init(effort_joint_interface, nh_pos2));
//  return (pos_ctrl1_.init(effort_joint_interface,controller_nh));
}
void RMPositionController::update(const ros::Time& time, const ros::Duration& period)
{
  error_ = pos_target_ - pos_ctrl1_.getPosition();
//  pos_ctrl1_.getGains(p_,temp_i_,d_,i_max_,i_min_,antiwindup_);
//  if (error_ > threshold_)
//    i_ = 0.0;
//  else
//    i_ = temp_i_;
//  pos_ctrl1_.setGains(p_,temp_i_,d_,i_max_,i_min_,antiwindup_);
//  pos_ctrl1_.setCommand(pos_target_);
//  pos_ctrl1_.update(time, period);
//  if (abs(error_) <= 0.2)
//    i_ = temp_i_;
//  else if (abs(error_) > 0.2 && abs(error_) < 0.3)
//    i_ = temp_i_ * (0.5 - abs(error_)) / 0.3;
//  else
//    i_ = 0.0;

  if (pos_ctrl1_.getPosition() > -0.5 || pos_ctrl2_.getPosition() > -0.5)
  {
    pos_ctrl1_.setCommand(pos_target_);
    pos_ctrl1_.update(time, period);
  }
  else
  {
    pos_ctrl2_.setCommand(pos_target_);
    pos_ctrl2_.update(time, period);
  }
}
}  // namespace rm_position_controller
PLUGINLIB_EXPORT_CLASS(rm_position_controller::RMPositionController, controller_interface::ControllerBase)