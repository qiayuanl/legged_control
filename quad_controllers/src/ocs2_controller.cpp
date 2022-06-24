//
// Created by qiayuan on 2022/6/24.
//
#include "quad_controllers/ocs2_controller.h"

#include <pluginlib/class_list_macros.hpp>
namespace quad_ros
{
bool Ocs2Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  std::string task_file, urdf_file, reference_file;
  controller_nh.getParam("/task_file", task_file);
  controller_nh.getParam("/urdf_file", urdf_file);
  controller_nh.getParam("/reference_file", reference_file);

  // Robot interface
  legged_interface_ = std::make_shared<LeggedRobotInterface>(task_file, urdf_file, reference_file);
  mpc_ = std::make_shared<MultipleShootingMpc>(legged_interface_->mpcSettings(), legged_interface_->sqpSettings(),
                                               legged_interface_->getOptimalControlProblem(),
                                               legged_interface_->getInitializer());
  return true;
}

void Ocs2Controller::update(const ros::Time& time, const ros::Duration& period)
{
}

}  // namespace quad_ros

PLUGINLIB_EXPORT_CLASS(quad_ros::Ocs2Controller, controller_interface::ControllerBase)
