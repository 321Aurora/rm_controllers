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

//
// Created by qiayuan on 5/16/21.
//

#include "rm_calibration_controllers/mechanical_calibration_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace rm_calibration_controllers
{
bool MechanicalCalibrationController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                           ros::NodeHandle& controller_nh)
{
  CalibrationBase::init(
      robot_hw, root_nh,
      controller_nh);  // 调用 CalibrationBase 类的 init 方法，传递机器人硬件接口、全局节点句柄和控制器节点句柄
  is_return_ = is_center_ = false;               // 初始化 is_return_ 和 is_center_ 为 false
  controller_nh.getParam("center", is_center_);  // 从控制器节点句柄获取参数 "center"，并将其值赋给 is_center_
  if (!controller_nh.getParam("velocity/vel_threshold",
                              velocity_threshold_))  // 从控制器节点句柄获取参数，如果获取失败则报错并返回 false
  {
    ROS_ERROR("Velocity threshold was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (velocity_threshold_ < 0)  // 如果速度阈值为负数，将其取绝对值并报错
  {
    velocity_threshold_ *= -1.;
    ROS_ERROR("Negative velocity threshold is not supported for joint %s. Making the velocity threshold positive.",
              velocity_ctrl_.getJointName().c_str());
  }
  if (controller_nh.hasParam("return"))  // 如果控制器节点句柄有参数 "return"
  {
    ros::NodeHandle nh_return(controller_nh, "return");  // 在控制器节点句柄下创建名为 "return" 的子节点句柄
    position_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(),
                        nh_return);  // 使用 "return" 子节点句柄初始化 position_ctrl_ 对象
    if (!nh_return.getParam("target_position",
                            target_position_))  // 从 "return" 子节点句柄获取参数，如果获取失败则报错并返回 false
    {
      ROS_ERROR("Position value was not specified (namespace: %s)", nh_return.getNamespace().c_str());
      return false;
    }
    if (!controller_nh.getParam("pos_threshold",
                                position_threshold_))  // 从控制器节点句柄获取参数，如果获取失败则报错并返回 false
    {
      ROS_ERROR("Position value was not specified (namespace: %s)", nh_return.getNamespace().c_str());
      return false;
    }
    is_return_ = true;             // 将 is_return_ 设置为 true，表示进行返回操作
    calibration_success_ = false;  // 将校准成功标志初始化为 false
  }
  return true;  // 初始化成功，返回 true
}

void MechanicalCalibrationController::update(const ros::Time& time, const ros::Duration& period)
{
  switch (state_)
  {
    case INITIALIZED:  // 在初始化状态下，设置速度命令为预定义的搜索速度，并初始化倒计时，将状态设为正向移动
    {
      velocity_ctrl_.setCommand(velocity_search_);
      countdown_ = 100;
      state_ = MOVING_POSITIVE;
      break;
    }
    case MOVING_POSITIVE:  // 在正向移动状态下，检查关节速度是否小于速度阈值且执行器未被停止
    {
      if (std::abs(velocity_ctrl_.joint_.getVelocity()) < velocity_threshold_ && !actuator_.getHalted())
        countdown_--;
      else
        countdown_ = 100;
      if (countdown_ < 0)  // 如果倒计时小于 0
      {
        velocity_ctrl_.setCommand(0);  // 设置速度命令为 0
        if (!is_center_)               // 如果不是居中校准
        {  // 调整执行器偏移，标记关节已校准，并根据是否需要返回设置下一状态
          actuator_.setOffset(-actuator_.getPosition() + actuator_.getOffset());
          actuator_.setCalibrated(true);
          ROS_INFO("Joint %s calibrated", velocity_ctrl_.getJointName().c_str());
          state_ = CALIBRATED;
          if (is_return_)
          {
            position_ctrl_.setCommand(target_position_);
          }
          else
          {
            velocity_ctrl_.joint_.setCommand(0.);
            calibration_success_ = true;
          }
        }
        else  // 如果是居中校准，记录正向移动时的位置，重新初始化倒计时，设置速度命令为反向
        {
          positive_position_ = actuator_.getPosition();
          countdown_ = 100;
          velocity_ctrl_.setCommand(-velocity_search_);
          state_ = MOVING_NEGATIVE;
        }
      }
      velocity_ctrl_.update(time, period);  // 更新速度控制器
      break;
    }
    case MOVING_NEGATIVE:
    {
      if (std::abs(velocity_ctrl_.joint_.getVelocity()) <
          velocity_threshold_)  // 在反向移动状态下，检查关节速度是否小于速度阈值
        countdown_--;
      else
        countdown_ = 100;
      if (countdown_ < 0)  // 如果倒计时小于 0
      {  // 设置速度命令为 0，记录反向移动时的位置，调整执行器偏移，标记关节已校准，并根据是否需要返回设置下一状态
        velocity_ctrl_.setCommand(0);
        negative_position_ = actuator_.getPosition();
        actuator_.setOffset(-(positive_position_ + negative_position_) / 2 + actuator_.getOffset());
        actuator_.setCalibrated(true);
        ROS_INFO("Joint %s calibrated", velocity_ctrl_.getJointName().c_str());
        state_ = CALIBRATED;
        if (is_return_)
          position_ctrl_.setCommand(target_position_);
        else
        {
          velocity_ctrl_.joint_.setCommand(0.);
          calibration_success_ = true;
        }
      }
      velocity_ctrl_.update(time, period);  // 更新速度控制器
      break;
    }
    case CALIBRATED:  // 在已校准状态下，根据是否需要返回判断校准成功，并更新位置或速度控制器
    {
      if (is_return_)
      {
        if ((std::abs(position_ctrl_.joint_.getPosition() - target_position_)) < position_threshold_)
          calibration_success_ = true;
        position_ctrl_.update(time, period);
      }
      else
        velocity_ctrl_.update(time, period);
      break;
    }
  }
}
}  // namespace rm_calibration_controllers

PLUGINLIB_EXPORT_CLASS(rm_calibration_controllers::MechanicalCalibrationController, controller_interface::ControllerBase)
