//
// Created by qiayuan on 2021/11/15.
//

#include "legged_estimation/StateEstimateBase.h"

#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include <utility>

namespace legged {
StateEstimateBase::StateEstimateBase(LeggedInterface& leggedInterface, std::vector<HybridJointHandle> hybridJointHandles,
                                     std::vector<ContactSensorHandle> contactSensorHandles,
                                     hardware_interface::ImuSensorHandle imuSensorHandle)
    : leggedInterface_(leggedInterface),
      generalizedCoordinatesNum_(leggedInterface.getCentroidalModelInfo().generalizedCoordinatesNum),
      rbdState_(2 * generalizedCoordinatesNum_),
      hybridJointHandles_(std::move(hybridJointHandles)),
      contactSensorHandles_(std::move(contactSensorHandles)),
      imuSensorHandle_(std::move(imuSensorHandle)) {
  ros::NodeHandle nh;
  odomPub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh, "odom", 10));
  odomPub_->msg_.header.frame_id = "odom";
  odomPub_->msg_.child_frame_id = "base";

  posePub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>(nh, "pose", 10));
  posePub_->msg_.header.frame_id = "odom";
}

size_t StateEstimateBase::getMode() {
  contact_flag_t contactFlag;
  for (size_t i = 0; i < contactSensorHandles_.size(); ++i) {
    contactFlag[i] = contactSensorHandles_[i].isContact();
  }

  return stanceLeg2ModeNumber(contactFlag);
}

void StateEstimateBase::updateAngular(const Eigen::Quaternion<scalar_t>& quat, const vector_t& angularVel) {
  rbdState_.segment<3>(0) = quatToZyx(quat);
  rbdState_.segment<3>(generalizedCoordinatesNum_) = angularVel;
}

void StateEstimateBase::updateLinear(const vector_t& pos, const vector_t& linearVel) {
  rbdState_.segment<3>(3) = pos;
  rbdState_.segment<3>(generalizedCoordinatesNum_ + 3) = linearVel;
}

void StateEstimateBase::updateJointStates() {
  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    rbdState_(6 + i) = hybridJointHandles_[i].getPosition();
    rbdState_(generalizedCoordinatesNum_ + 6 + i) = hybridJointHandles_[i].getVelocity();
  }
}

void StateEstimateBase::publishMsgs(const nav_msgs::Odometry& odom, const ros::Time& time) {
  scalar_t publishRate = 100;
  if (lastPub_ + ros::Duration(1. / publishRate) < time) {
    if (odomPub_->trylock()) {
      odomPub_->msg_.header.stamp = time;
      odomPub_->msg_.pose = odom.pose;
      odomPub_->msg_.twist = odom.twist;
      odomPub_->unlockAndPublish();
    }
    if (posePub_->trylock()) {
      posePub_->msg_.header.stamp = time;
      posePub_->msg_.pose = odom.pose;
      posePub_->unlockAndPublish();
    }
  }
}

}  // namespace legged
