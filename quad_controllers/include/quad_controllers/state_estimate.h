//
// Created by qiayuan on 2021/11/15.
//

#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <quad_common/hardware_interface/hybrid_joint_interface.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>

namespace quad_ros
{
using namespace ocs2;

class StateEstimateBase
{
public:
  StateEstimateBase(ros::NodeHandle& nh, PinocchioInterface& pinocchio_interface,
                    const CentroidalModelInfo& centroidal_model_info,
                    const std::vector<HybridJointHandle>& hybrid_joint_handles);
  virtual ~StateEstimateBase(){};
  virtual vector_t update() = 0;

protected:
  PinocchioInterface& pinocchio_interface_;
  const CentroidalModelInfo& centroidal_model_info_;
  CentroidalModelRbdConversions centroidal_conversions_;
  const std::vector<HybridJointHandle>& hybrid_joint_handles_;

private:
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
  ros::Time last_publish_;
};

class FromTopicStateEstimate : public StateEstimateBase
{
public:
  FromTopicStateEstimate(ros::NodeHandle& nh, PinocchioInterface& pinocchio_interface,
                         const CentroidalModelInfo& centroidal_model_info,
                         const std::vector<HybridJointHandle>& hybrid_joint_handles);
  ~FromTopicStateEstimate() override{};

  vector_t update() override;

private:
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
};

template <typename T>
T square(T a)
{
  return a * a;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q)
{
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) =
      std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) =
      std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

}  // namespace quad_ros
