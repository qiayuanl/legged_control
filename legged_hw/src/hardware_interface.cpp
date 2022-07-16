
//
// Created by qiayuan on 1/24/22.
//

#include "legged_hw/hardware_interface.h"

namespace legged
{
bool QuadHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  if (!loadUrdf(root_nh))
  {
    ROS_ERROR("Error occurred while setting up urdf");
    return false;
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&hybrid_joint_interface_);
  registerInterface(&imu_sensor_interface_);
  registerInterface(&contact_sensor_interface_);

  return true;
}

bool QuadHW::loadUrdf(ros::NodeHandle& root_nh)
{
  std::string urdf_string;
  if (urdf_model_ == nullptr)
    urdf_model_ = std::make_shared<urdf::Model>();
  // get the urdf param on param server
  root_nh.getParam("legged_robot_description", urdf_string);
  return !urdf_string.empty() && urdf_model_->initString(urdf_string);
}

}  // namespace legged
