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
#include <quad_common/hardware_interface/contact_sensor_interface.h>

#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

namespace quad_ros
{
using namespace ocs2;
using namespace legged_robot;

class StateEstimateBase
{
public:
  StateEstimateBase(ros::NodeHandle& nh, LeggedRobotInterface& legged_interface,
                    const std::vector<HybridJointHandle>& hybrid_joint_handles,
                    const std::vector<ContactSensorHandle>& contact_sensor_handles,
                    const hardware_interface::ImuSensorHandle& imu_sensor_handle);
  virtual vector_t update(scalar_t dt) = 0;
  size_t getMode();

protected:
  void updateAngular(const Eigen::Quaternion<scalar_t>& quat, const vector_t& angular_vel);
  void updateLinear(const vector_t& pos, const vector_t& linear_vel);
  void updateJointStates();

  LeggedRobotInterface& legged_interface_;
  size_t generalized_coordinates_num_;
  vector_t rbd_state_;

  const std::vector<HybridJointHandle> hybrid_joint_handles_;
  const std::vector<ContactSensorHandle> contact_sensor_handles_;
  const hardware_interface::ImuSensorHandle imu_sensor_handle_;

private:
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
  ros::Time last_publish_;
};

class FromTopicStateEstimate : public StateEstimateBase
{
public:
  FromTopicStateEstimate(ros::NodeHandle& nh, LeggedRobotInterface& legged_interface,
                         const std::vector<HybridJointHandle>& hybrid_joint_handles,
                         const std::vector<ContactSensorHandle>& contact_sensor_handles,
                         const hardware_interface::ImuSensorHandle& imu_sensor_handle);

  vector_t update(scalar_t dt) override;

private:
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
};

class KalmanFilterEstimate : public StateEstimateBase
{
public:
  KalmanFilterEstimate(ros::NodeHandle& nh, LeggedRobotInterface& legged_interface,
                       const std::vector<HybridJointHandle>& hybrid_joint_handles,
                       const std::vector<ContactSensorHandle>& contact_sensor_handles,
                       const hardware_interface::ImuSensorHandle& imu_sensor_handle);
  vector_t update(scalar_t dt) override;

private:
  PinocchioEndEffectorKinematics pinocchio_ee_kine_;

  Eigen::Matrix<scalar_t, 18, 1> x_hat_;
  Eigen::Matrix<scalar_t, 12, 1> ps_;
  Eigen::Matrix<scalar_t, 12, 1> vs_;
  Eigen::Matrix<scalar_t, 18, 18> a_;
  Eigen::Matrix<scalar_t, 18, 18> q_;
  Eigen::Matrix<scalar_t, 18, 18> p_;
  Eigen::Matrix<scalar_t, 28, 28> r_;
  Eigen::Matrix<scalar_t, 18, 3> b_;
  Eigen::Matrix<scalar_t, 28, 18> c_;
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
