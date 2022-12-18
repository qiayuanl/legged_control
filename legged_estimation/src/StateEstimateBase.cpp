//
// Created by qiayuan on 2021/11/15.
//

#include "legged_estimation/StateEstimateBase.h"

#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include <utility>

namespace legged {
StateEstimateBase::StateEstimateBase(LeggedInterface& legged_interface, std::vector<HybridJointHandle> hybrid_joint_handles,
                                     std::vector<ContactSensorHandle> contact_sensor_handles,
                                     hardware_interface::ImuSensorHandle imu_sensor_handle)
    : leggedInterface_(legged_interface),
      generalizedCoordinatesNum_(legged_interface.getCentroidalModelInfo().generalizedCoordinatesNum),
      rbdState_(2 * generalizedCoordinatesNum_),
      hybridJointHandles_(std::move(hybrid_joint_handles)),
      contactSensorHandles_(std::move(contact_sensor_handles)),
      imuSensorHandle_(std::move(imu_sensor_handle)) {
  ros::NodeHandle nh;
  odomPub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh, "odom", 10));
  odomPub_->msg_.header.frame_id = "odom";
  odomPub_->msg_.child_frame_id = "base";

  posePub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>(nh, "pose", 10));
  posePub_->msg_.header.frame_id = "odom";
}

size_t StateEstimateBase::getMode() {
  contact_flag_t contact_flag;
  for (size_t i = 0; i < contactSensorHandles_.size(); ++i) {
    contact_flag[i] = contactSensorHandles_[i].isContact();
  }

  return stanceLeg2ModeNumber(contact_flag);
}

void StateEstimateBase::updateAngular(const Eigen::Quaternion<scalar_t>& quat, const vector_t& angular_vel) {
  rbdState_.segment<3>(0) = quatToZyx(quat);
  rbdState_.segment<3>(generalizedCoordinatesNum_) = angular_vel;
}

void StateEstimateBase::updateLinear(const vector_t& pos, const vector_t& linear_vel) {
  rbdState_.segment<3>(3) = pos;
  rbdState_.segment<3>(generalizedCoordinatesNum_ + 3) = linear_vel;
}

void StateEstimateBase::updateJointStates() {
  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    rbdState_(6 + i) = hybridJointHandles_[i].getPosition();
    rbdState_(generalizedCoordinatesNum_ + 6 + i) = hybridJointHandles_[i].getVelocity();
  }
}

void StateEstimateBase::publishMsgs(const nav_msgs::Odometry& odom, const ros::Time& time) {
  scalar_t publish_rate = 100;
  if (lastPub_ + ros::Duration(1. / publish_rate) < time) {
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
