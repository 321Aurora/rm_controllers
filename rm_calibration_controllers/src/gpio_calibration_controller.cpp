//
// Created by guanlin on 22-11-7.
//

#include "rm_calibration_controllers/gpio_calibration_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace rm_calibration_controllers
{
bool GpioCalibrationController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                                     ros::NodeHandle& controller_nh)
{
  CalibrationBase::init(
      robot_hw, root_nh,
      controller_nh);  // 调用 CalibrationBase 类的 init 方法，传递机器人硬件接口、全局节点句柄和控制器节点句柄
  ros::NodeHandle pos_nh(controller_nh, "position");  // 在控制器节点句柄下创建一个名为 "position" 的子节点句柄
  position_ctrl_.init(robot_hw->get<hardware_interface::EffortJointInterface>(),
                      pos_nh);  // 调用 position_ctrl_ 对象的 init 方法，传递机器人硬件接口和 "position" 子节点句柄
  if (!pos_nh.getParam("pos_threshold", position_threshold_))  // 从节点句柄中获取参数，如果获取失败则报错并返回 false
  {
    ROS_ERROR("Position threshold was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!pos_nh.getParam("backward_angle", backward_angle_))
  {
    ROS_ERROR("Backward angle was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("velocity/slow_forward_velocity", slow_forward_velocity_))
  {
    ROS_ERROR("Slow forward velocity was not specified (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  std::string gpio{};
  if (!controller_nh.getParam("gpio", gpio))
  {
    ROS_ERROR("No gpio given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  if (!controller_nh.getParam("initial_gpio_state", initial_gpio_state_))
  {
    ROS_ERROR("No initial gpio states given (namespace: %s)", controller_nh.getNamespace().c_str());
    return false;
  }
  gpio_state_handle_ = robot_hw->get<rm_control::GpioStateInterface>()->getHandle(
      gpio);    // 通过机器人硬件接口获取 GpioStateInterface，并从中获取指定 GPIO 的状态句柄
  return true;  // 获取成功
}

void GpioCalibrationController::update(const ros::Time& time, const ros::Duration& period)
{
  switch (state_)
  {
    case INITIALIZED:  // 在初始化状态下，设置速度命令为预定义的搜索速度，并切换到 FAST_FORWARD 状态
    {
      velocity_ctrl_.setCommand(velocity_search_);
      state_ = FAST_FORWARD;
      break;
    }
    case FAST_FORWARD:
    {  // 在快速前进状态下，检查 GPIO 状态是否与初始状态不同
      if (gpio_state_handle_.getValue() !=
          initial_gpio_state_)  // 如果不同，记录开始后退时的关节位置，将速度命令设置为 0，并切换到 RETREAT 状态
      {
        start_retreat_position_ = velocity_ctrl_.joint_.getPosition();
        velocity_ctrl_.setCommand(0);
        state_ = RETREAT;
      }
      else  // 如果 GPIO 状态相同，更新速度控制器
        velocity_ctrl_.update(time, period);
      break;
    }
    case RETREAT:
    {  // 在后退状态下，设置位置命令为开始后退的位置减去后退角度
      position_ctrl_.setCommand(start_retreat_position_ - backward_angle_);
      position_ctrl_.update(time, period);
      if (std::abs(position_ctrl_.command_struct_.position_ - position_ctrl_.joint_.getPosition()) <
          position_threshold_)  // 如果当前关节位置与命令位置的差小于位置阈值，切换到 SLOW_FORWARD 状态
        state_ = SLOW_FORWARD;
      break;
    }
    case SLOW_FORWARD:
    {  // 在慢速前进状态下，设置速度命令为慢速前进速度
      velocity_ctrl_.setCommand(slow_forward_velocity_);
      if (gpio_state_handle_.getValue() != initial_gpio_state_)  // 如果 GPIO 状态与初始状态不同
      {  // 设置速度命令为 0，调整执行器偏移，并标记关节已校准，然后切换到 CALIBRATED 状态
        velocity_ctrl_.setCommand(0);
        actuator_.setOffset(-actuator_.getPosition() + actuator_.getOffset());
        actuator_.setCalibrated(true);
        ROS_INFO("Joint %s calibrated", velocity_ctrl_.getJointName().c_str());
        state_ = CALIBRATED;
      }  // 更新速度控制器
      velocity_ctrl_.update(time, period);
      break;
    }
    case CALIBRATED:  // 在已校准状态下，设置校准成功标志为 true
      calibration_success_ = true;
  }
}
}  // namespace rm_calibration_controllers

PLUGINLIB_EXPORT_CLASS(rm_calibration_controllers::GpioCalibrationController, controller_interface::ControllerBase)
