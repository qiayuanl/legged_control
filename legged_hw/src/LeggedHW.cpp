
//
// Created by qiayuan on 1/24/22.
//

#include "legged_hw/LeggedHW.h"

namespace legged {
bool LeggedHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& /*robot_hw_nh*/) {
  if (!loadUrdf(root_nh)) {
    ROS_ERROR("Error occurred while setting up urdf");
    return false;
  }

  registerInterface(&jointStateInterface_);
  registerInterface(&hybridJointInterface_);
  registerInterface(&imuSensorInterface_);
  registerInterface(&contactSensorInterface_);

  return true;
}

bool LeggedHW::loadUrdf(ros::NodeHandle& rootNh) {
  std::string urdfString;
  if (urdfModel_ == nullptr) {
    urdfModel_ = std::make_shared<urdf::Model>();
  }
  // get the urdf param on param server
  rootNh.getParam("legged_robot_description", urdfString);
  return !urdfString.empty() && urdfModel_->initString(urdfString);
}

}  // namespace legged
