//
// Created by qiayuan on 2021/11/15.
//
#pragma once

#include <ros/ros.h>

#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/hybrid_joint_interface.h>
#include <legged_common/hardware_interface/contact_sensor_interface.h>
#include <legged_interface/legged_interface.h>

namespace legged
{
using namespace ocs2;
using namespace legged_robot;

class StateEstimateBase
{
public:
  StateEstimateBase(LeggedInterface& legged_interface, const std::vector<HybridJointHandle>& hybrid_joint_handles,
                    const std::vector<ContactSensorHandle>& contact_sensor_handles,
                    const hardware_interface::ImuSensorHandle& imu_sensor_handle);
  virtual vector_t update(scalar_t dt) = 0;
  size_t getMode();

protected:
  void updateAngular(const Eigen::Quaternion<scalar_t>& quat, const vector_t& angular_vel);
  void updateLinear(const vector_t& pos, const vector_t& linear_vel);
  void updateJointStates();

  LeggedInterface& legged_interface_;
  size_t generalized_coordinates_num_;
  vector_t rbd_state_;

  const std::vector<HybridJointHandle> hybrid_joint_handles_;
  const std::vector<ContactSensorHandle> contact_sensor_handles_;
  const hardware_interface::ImuSensorHandle imu_sensor_handle_;
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

}  // namespace legged
