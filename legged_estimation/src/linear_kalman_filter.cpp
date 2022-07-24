//
// Created by qiayuan on 2022/7/24.
//

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "legged_estimation/linear_kalman_filter.h"

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace legged
{
KalmanFilterEstimate::KalmanFilterEstimate(LeggedInterface& legged_interface,
                                           const std::vector<HybridJointHandle>& hybrid_joint_handles,
                                           const std::vector<ContactSensorHandle>& contact_sensor_handles,
                                           const hardware_interface::ImuSensorHandle& imu_sensor_handle)
  : StateEstimateBase(legged_interface, hybrid_joint_handles, contact_sensor_handles, imu_sensor_handle)
  , pinocchio_ee_kine_(legged_interface.getPinocchioInterface(),
                       CentroidalModelPinocchioMapping(legged_interface.getCentroidalModelInfo()),
                       legged_interface.modelSettings().contactNames3DoF)
{
  x_hat_.setZero();
  ps_.setZero();
  vs_.setZero();
  a_.setZero();
  a_.block(0, 0, 3, 3) = Eigen::Matrix<scalar_t, 3, 3>::Identity();
  a_.block(3, 3, 3, 3) = Eigen::Matrix<scalar_t, 3, 3>::Identity();
  a_.block(6, 6, 12, 12) = Eigen::Matrix<scalar_t, 12, 12>::Identity();
  b_.setZero();

  Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> c1(3, 6);
  c1 << Eigen::Matrix<scalar_t, 3, 3>::Identity(), Eigen::Matrix<scalar_t, 3, 3>::Zero();
  Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> c2(3, 6);
  c2 << Eigen::Matrix<scalar_t, 3, 3>::Zero(), Eigen::Matrix<scalar_t, 3, 3>::Identity();
  c_.setZero();
  c_.block(0, 0, 3, 6) = c1;
  c_.block(3, 0, 3, 6) = c1;
  c_.block(6, 0, 3, 6) = c1;
  c_.block(9, 0, 3, 6) = c1;
  c_.block(0, 6, 12, 12) = Eigen::Matrix<scalar_t, 12, 12>::Identity();
  c_.block(12, 0, 3, 6) = c2;
  c_.block(15, 0, 3, 6) = c2;
  c_.block(18, 0, 3, 6) = c2;
  c_.block(21, 0, 3, 6) = c2;
  c_(27, 17) = 1.0;
  c_(26, 14) = 1.0;
  c_(25, 11) = 1.0;
  c_(24, 8) = 1.0;
  p_.setIdentity();
  p_ = 100. * p_;
  q_.setIdentity();
  r_.setIdentity();
}

vector_t KalmanFilterEstimate::update(scalar_t dt)
{
  a_.block(0, 3, 3, 3) = dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  b_.block(0, 0, 3, 3) = 0.5 * dt * dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  b_.block(3, 0, 3, 3) = dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  q_.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  q_.block(6, 6, 12, 12) = dt * Eigen::Matrix<scalar_t, 12, 12>::Identity();

  // Angular from IMU
  Eigen::Quaternion<scalar_t> quat(imu_sensor_handle_.getOrientation()[3], imu_sensor_handle_.getOrientation()[0],
                                   imu_sensor_handle_.getOrientation()[1], imu_sensor_handle_.getOrientation()[2]);
  Eigen::Matrix<scalar_t, 3, 1> angular_vel_local(imu_sensor_handle_.getAngularVelocity()[0],
                                                  imu_sensor_handle_.getAngularVelocity()[1],
                                                  imu_sensor_handle_.getAngularVelocity()[2]);
  vector_t zyx = quatToZyx(quat);

  updateAngular(quat, getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
                          zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(zyx, angular_vel_local)));

  // Joint states
  updateJointStates();

  auto& model = legged_interface_.getPinocchioInterface().getModel();
  auto& data = legged_interface_.getPinocchioInterface().getData();
  size_t actuated_dof_num = legged_interface_.getCentroidalModelInfo().actuatedDofNum;

  vector_t q_pino(generalized_coordinates_num_), v_pino(generalized_coordinates_num_);
  q_pino.setZero();
  q_pino.segment<3>(3) = rbd_state_.head<3>();  // Only set orientation, let position in origin.
  q_pino.tail(actuated_dof_num) = rbd_state_.segment(6, actuated_dof_num);

  v_pino.setZero();
  v_pino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      q_pino.segment<3>(3),
      rbd_state_.segment<3>(legged_interface_.getCentroidalModelInfo()
                                .generalizedCoordinatesNum));  // Only set angular velocity, let linear velocity be zero
  v_pino.tail(actuated_dof_num) = rbd_state_.segment(6 + generalized_coordinates_num_, actuated_dof_num);

  pinocchio::forwardKinematics(model, data, q_pino, v_pino);
  pinocchio::updateFramePlacements(legged_interface_.getPinocchioInterface().getModel(),
                                   legged_interface_.getPinocchioInterface().getData());
  pinocchio_ee_kine_.setPinocchioInterface(legged_interface_.getPinocchioInterface());

  const auto ee_pos = pinocchio_ee_kine_.getPosition(vector_t());
  const auto ee_vel = pinocchio_ee_kine_.getVelocity(vector_t(), vector_t());

  scalar_t imu_process_noise_position = 0.2;
  scalar_t imu_process_noise_velocity = 0.2;
  scalar_t foot_process_noise_position = 0.002;
  scalar_t foot_sensor_noise_position = 0.005;
  scalar_t foot_sensor_noise_velocity = 0.1;  // TODO adjust the value
  scalar_t foot_height_sensor_noise = 0.001;
  Eigen::Matrix<scalar_t, 18, 18> q = Eigen::Matrix<scalar_t, 18, 18>::Identity();
  q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imu_process_noise_position;
  q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imu_process_noise_velocity;
  q.block(6, 6, 12, 12) = q_.block(6, 6, 12, 12) * foot_process_noise_position;

  Eigen::Matrix<scalar_t, 28, 28> r = Eigen::Matrix<scalar_t, 28, 28>::Identity();
  r.block(0, 0, 12, 12) = r_.block(0, 0, 12, 12) * foot_sensor_noise_position;
  r.block(12, 12, 12, 12) = r_.block(12, 12, 12, 12) * foot_sensor_noise_velocity;
  r.block(24, 24, 4, 4) = r_.block(24, 24, 4, 4) * foot_height_sensor_noise;

  for (int i = 0; i < 4; i++)
  {
    int i1 = 3 * i;

    int q_index = 6 + i1;
    int r_index1 = i1;
    int r_index2 = 12 + i1;
    int r_index3 = 24 + i;
    bool is_contact = contact_sensor_handles_[i].isContact();

    scalar_t high_suspect_number(100);
    q.block(q_index, q_index, 3, 3) = (is_contact ? 1. : high_suspect_number) * q.block(q_index, q_index, 3, 3);
    r.block(r_index1, r_index1, 3, 3) = 1. * r.block(r_index1, r_index1, 3, 3);
    r.block(r_index2, r_index2, 3, 3) = (is_contact ? 1. : high_suspect_number) * r.block(r_index2, r_index2, 3, 3);
    r(r_index3, r_index3) = (is_contact ? 1. : high_suspect_number) * r(r_index3, r_index3);

    scalar_t foot_radius = 0.02;
    ps_.segment(3 * i, 3) = -1. * ee_pos[i];
    ps_.segment(3 * i, 3)[2] += foot_radius;
    vs_.segment(3 * i, 3) = -1. * ee_vel[i];
  }
  Eigen::Matrix<scalar_t, 3, 1> g(0, 0, -9.81);
  Eigen::Matrix<scalar_t, 3, 1> imu_accel(imu_sensor_handle_.getLinearAcceleration()[0],
                                          imu_sensor_handle_.getLinearAcceleration()[1],
                                          imu_sensor_handle_.getLinearAcceleration()[2]);
  Eigen::Matrix<scalar_t, 3, 1> accel = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat)) * imu_accel + g;
  Eigen::Matrix<scalar_t, 4, 1> pzs = Eigen::Matrix<scalar_t, 4, 1>::Zero();

  Eigen::Matrix<scalar_t, 28, 1> y;
  y << ps_, vs_, pzs;
  x_hat_ = a_ * x_hat_ + b_ * accel;
  Eigen::Matrix<scalar_t, 18, 18> at = a_.transpose();
  Eigen::Matrix<scalar_t, 18, 18> pm = a_ * p_ * at + q;
  Eigen::Matrix<scalar_t, 18, 28> ct = c_.transpose();
  Eigen::Matrix<scalar_t, 28, 1> y_model = c_ * x_hat_;
  Eigen::Matrix<scalar_t, 28, 1> ey = y - y_model;
  Eigen::Matrix<scalar_t, 28, 28> s = c_ * pm * ct + r;

  Eigen::Matrix<scalar_t, 28, 1> s_ey = s.lu().solve(ey);
  x_hat_ += pm * ct * s_ey;

  Eigen::Matrix<scalar_t, 28, 18> s_c = s.lu().solve(c_);
  p_ = (Eigen::Matrix<scalar_t, 18, 18>::Identity() - pm * ct * s_c) * pm;

  Eigen::Matrix<scalar_t, 18, 18> pt = p_.transpose();
  p_ = (p_ + pt) / 2.0;

  if (p_.block(0, 0, 2, 2).determinant() > 0.000001)
  {
    p_.block(0, 2, 2, 16).setZero();
    p_.block(2, 0, 16, 2).setZero();
    p_.block(0, 0, 2, 2) /= 10.;
  }
  updateLinear(x_hat_.block(0, 0, 3, 1), x_hat_.block(3, 0, 3, 1));
  return rbd_state_;
}

}  // namespace legged
