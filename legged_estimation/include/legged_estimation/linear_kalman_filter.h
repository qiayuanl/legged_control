//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "legged_estimation/state_estimate_base.h"

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>

namespace legged
{
using namespace ocs2;
using namespace legged_robot;

class KalmanFilterEstimate : public StateEstimateBase
{
public:
  KalmanFilterEstimate(LeggedInterface& legged_interface, const std::vector<HybridJointHandle>& hybrid_joint_handles,
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

}  // namespace legged