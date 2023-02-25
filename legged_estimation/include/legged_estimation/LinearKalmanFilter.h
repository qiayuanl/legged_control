//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "legged_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <realtime_tools/realtime_buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace legged {
using namespace ocs2;

class KalmanFilterEstimate : public StateEstimateBase {
 public:
  KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics,
                       const std::vector<HybridJointHandle>& hybridJointHandles,
                       const std::vector<ContactSensorHandle>& contactSensorHandles,
                       const hardware_interface::ImuSensorHandle& imuSensorHandle);
  vector_t update(const ros::Time& time, const ros::Duration& period) override;

 private:
  Eigen::Quaternion<scalar_t> getQuat() {
    return Eigen::Quaternion<scalar_t>(imuSensorHandle_.getOrientation()[3], imuSensorHandle_.getOrientation()[0],
                                       imuSensorHandle_.getOrientation()[1], imuSensorHandle_.getOrientation()[2]);
  }

  Eigen::Matrix<scalar_t, 3, 1> getAngularVelLocal() {
    return Eigen::Matrix<scalar_t, 3, 1>(imuSensorHandle_.getAngularVelocity()[0], imuSensorHandle_.getAngularVelocity()[1],
                                         imuSensorHandle_.getAngularVelocity()[2]);
  }

  void updateFromTopic();

  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  nav_msgs::Odometry getOdomMsg();

  Eigen::Matrix<scalar_t, 18, 1> xHat_;
  Eigen::Matrix<scalar_t, 12, 1> ps_;
  Eigen::Matrix<scalar_t, 12, 1> vs_;
  Eigen::Matrix<scalar_t, 18, 18> a_;
  Eigen::Matrix<scalar_t, 18, 18> q_;
  Eigen::Matrix<scalar_t, 18, 18> p_;
  Eigen::Matrix<scalar_t, 28, 28> r_;
  Eigen::Matrix<scalar_t, 18, 3> b_;
  Eigen::Matrix<scalar_t, 28, 18> c_;

  scalar_t footRadius_ = 0.02;
  vector_t feetHeights_;
  vector3_t zyxOffset_ = vector3_t::Zero();

  // Topic
  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2::Transform world2odom_;
  std::string frameOdom_, frameGuess_;
  bool topicUpdated_;
};

}  // namespace legged