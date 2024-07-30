#pragma once

#include <effort_controllers/joint_position_controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Float64.h>

#include <pluginlib/class_list_macros.hpp>

namespace rm_position_controller
{

/*!
 * Class containing the algorithmic part of the package.
 */
class RMPositionController
  : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                          hardware_interface::JointStateInterface>
{
public:
  RMPositionController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  void commandCB(const std_msgs::Float64ConstPtr& msg)
  {
    pos_target_ = msg->data;
  }
  effort_controllers::JointPositionController pos_ctrl1_, pos_ctrl2_;
  hardware_interface::JointStateHandle target_state_handle_;
  double error_{};
  control_toolbox::Pid pid1_, pid2_;
  double p_, i_, d_, i_max_, i_min_, pos_target_, threshold_, temp_i_, temp_p_;
  bool antiwindup_;
  ros::Subscriber cmd_subscriber_;
};

}  // namespace rm_position_controller
