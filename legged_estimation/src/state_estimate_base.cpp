//
// Created by qiayuan on 2021/11/15.
//

#include "legged_estimation/state_estimate_base.h"

#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

namespace legged
{
StateEstimateBase::StateEstimateBase(LeggedInterface& legged_interface,
                                     const std::vector<HybridJointHandle>& hybrid_joint_handles,
                                     const std::vector<ContactSensorHandle>& contact_sensor_handles,
                                     const hardware_interface::ImuSensorHandle& imu_sensor_handle)
  : legged_interface_(legged_interface)
  , generalized_coordinates_num_(legged_interface.getCentroidalModelInfo().generalizedCoordinatesNum)
  , rbd_state_(2 * generalized_coordinates_num_)
  , hybrid_joint_handles_(hybrid_joint_handles)
  , contact_sensor_handles_(contact_sensor_handles)
  , imu_sensor_handle_(imu_sensor_handle)
{
}

size_t StateEstimateBase::getMode()
{
  contact_flag_t contact_flag;
  for (size_t i = 0; i < contact_sensor_handles_.size(); ++i)
    contact_flag[i] = contact_sensor_handles_[i].isContact();

  return stanceLeg2ModeNumber(contact_flag);
}

void StateEstimateBase::updateAngular(const Eigen::Quaternion<scalar_t>& quat, const vector_t& angular_vel)
{
  rbd_state_.segment<3>(0) = quatToZyx(quat);
  rbd_state_.segment<3>(generalized_coordinates_num_) = angular_vel;
}

void StateEstimateBase::updateLinear(const vector_t& pos, const vector_t& linear_vel)
{
  rbd_state_.segment<3>(3) = pos;
  rbd_state_.segment<3>(generalized_coordinates_num_ + 3) = linear_vel;
}

void StateEstimateBase::updateJointStates()
{
  for (size_t i = 0; i < hybrid_joint_handles_.size(); ++i)
  {
    rbd_state_(6 + i) = hybrid_joint_handles_[i].getPosition();
    rbd_state_(generalized_coordinates_num_ + 6 + i) = hybrid_joint_handles_[i].getVelocity();
  }
}

}  // namespace legged
