//
// Created by qiayuan on 2021/11/15.
//
#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>

#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include <legged_interface/LeggedInterface.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class StateEstimateBase {
 public:
  StateEstimateBase(LeggedInterface& legged_interface, std::vector<HybridJointHandle> hybrid_joint_handles,
                    std::vector<ContactSensorHandle> contact_sensor_handles, hardware_interface::ImuSensorHandle imu_sensor_handle);
  virtual vector_t update(const ros::Time& time, const ros::Duration& period) = 0;
  size_t getMode();

 protected:
  void updateAngular(const Eigen::Quaternion<scalar_t>& quat, const vector_t& angular_vel);
  void updateLinear(const vector_t& pos, const vector_t& linear_vel);
  void updateJointStates();
  void publishMsgs(const nav_msgs::Odometry& odom, const ros::Time& time);

  LeggedInterface& leggedInterface_;
  size_t generalizedCoordinatesNum_;
  vector_t rbdState_;

  const std::vector<HybridJointHandle> hybridJointHandles_;
  const std::vector<ContactSensorHandle> contactSensorHandles_;
  const hardware_interface::ImuSensorHandle imuSensorHandle_;

  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odomPub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>> posePub_;
  ros::Time lastPub_;
};

template <typename T>
T square(T a) {
  return a * a;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) {
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

}  // namespace legged