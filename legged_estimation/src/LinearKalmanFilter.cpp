//
// Created by qiayuan on 2022/7/24.
//

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_estimation/LinearKalmanFilter.h"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace legged {
KalmanFilterEstimate::KalmanFilterEstimate(LeggedInterface& legged_interface, const std::vector<HybridJointHandle>& hybrid_joint_handles,
                                           const std::vector<ContactSensorHandle>& contact_sensor_handles,
                                           const hardware_interface::ImuSensorHandle& imu_sensor_handle)
    : StateEstimateBase(legged_interface, hybrid_joint_handles, contact_sensor_handles, imu_sensor_handle),
      pinocchioEeKine_(legged_interface.getPinocchioInterface(), CentroidalModelPinocchioMapping(legged_interface.getCentroidalModelInfo()),
                       legged_interface.modelSettings().contactNames3DoF),
      tfListener_(tfBuffer_),
      topicUpdated_(false) {
  xHat_.setZero();
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
  c_.block(0, 6, 12, 12) = -Eigen::Matrix<scalar_t, 12, 12>::Identity();
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

  world2odom_.setRotation(tf2::Quaternion::getIdentity());
  sub_ = ros::NodeHandle().subscribe<nav_msgs::Odometry>("/tracking_camera/odom/sample", 10, &KalmanFilterEstimate::callback, this);
}

vector_t KalmanFilterEstimate::update(const ros::Time& time, const ros::Duration& period) {
  // Angular from IMU
  Eigen::Quaternion<scalar_t> quat(imuSensorHandle_.getOrientation()[3], imuSensorHandle_.getOrientation()[0],
                                   imuSensorHandle_.getOrientation()[1], imuSensorHandle_.getOrientation()[2]);
  Eigen::Matrix<scalar_t, 3, 1> angular_vel_local(imuSensorHandle_.getAngularVelocity()[0], imuSensorHandle_.getAngularVelocity()[1],
                                                  imuSensorHandle_.getAngularVelocity()[2]);
  vector_t zyx = quatToZyx(quat);
  Eigen::Matrix<scalar_t, 3, 1> angular_vel_global = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
      zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(zyx, angular_vel_local));
  updateAngular(quat, angular_vel_global);

  // Joint states
  updateJointStates();

  scalar_t dt = period.toSec();
  a_.block(0, 3, 3, 3) = dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  b_.block(0, 0, 3, 3) = 0.5 * dt * dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  b_.block(3, 0, 3, 3) = dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  q_.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * Eigen::Matrix<scalar_t, 3, 3>::Identity();
  q_.block(6, 6, 12, 12) = dt * Eigen::Matrix<scalar_t, 12, 12>::Identity();

  const auto& model = leggedInterface_.getPinocchioInterface().getModel();
  auto& data = leggedInterface_.getPinocchioInterface().getData();
  size_t actuated_dof_num = leggedInterface_.getCentroidalModelInfo().actuatedDofNum;

  vector_t q_pino(generalizedCoordinatesNum_);
  vector_t v_pino(generalizedCoordinatesNum_);
  q_pino.setZero();
  q_pino.segment<3>(3) = rbdState_.head<3>();  // Only set orientation, let position in origin.
  q_pino.tail(actuated_dof_num) = rbdState_.segment(6, actuated_dof_num);

  v_pino.setZero();
  v_pino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      q_pino.segment<3>(3),
      rbdState_.segment<3>(
          leggedInterface_.getCentroidalModelInfo().generalizedCoordinatesNum));  // Only set angular velocity, let linear velocity be zero
  v_pino.tail(actuated_dof_num) = rbdState_.segment(6 + generalizedCoordinatesNum_, actuated_dof_num);

  pinocchio::forwardKinematics(model, data, q_pino, v_pino);
  pinocchio::updateFramePlacements(leggedInterface_.getPinocchioInterface().getModel(), leggedInterface_.getPinocchioInterface().getData());
  pinocchioEeKine_.setPinocchioInterface(leggedInterface_.getPinocchioInterface());

  const auto ee_pos = pinocchioEeKine_.getPosition(vector_t());
  const auto ee_vel = pinocchioEeKine_.getVelocity(vector_t(), vector_t());

  scalar_t foot_radius = 0.015;
  scalar_t imu_process_noise_position = 0.2;
  scalar_t imu_process_noise_velocity = 0.2;
  scalar_t foot_process_noise_position = 0.002;
  scalar_t foot_sensor_noise_position = 0.005;
  scalar_t foot_sensor_noise_velocity = 0.1;  // TODO(qiayuan): adjust the value
  scalar_t foot_height_sensor_noise = 0.005;

  Eigen::Matrix<scalar_t, 18, 18> q = Eigen::Matrix<scalar_t, 18, 18>::Identity();
  q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imu_process_noise_position;
  q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imu_process_noise_velocity;
  q.block(6, 6, 12, 12) = q_.block(6, 6, 12, 12) * foot_process_noise_position;

  Eigen::Matrix<scalar_t, 28, 28> r = Eigen::Matrix<scalar_t, 28, 28>::Identity();
  r.block(0, 0, 12, 12) = r_.block(0, 0, 12, 12) * foot_sensor_noise_position;
  r.block(12, 12, 12, 12) = r_.block(12, 12, 12, 12) * foot_sensor_noise_velocity;
  r.block(24, 24, 4, 4) = r_.block(24, 24, 4, 4) * foot_height_sensor_noise;

  for (int i = 0; i < 4; i++) {
    int i1 = 3 * i;

    int q_index = 6 + i1;
    int r_index1 = i1;
    int r_index2 = 12 + i1;
    int r_index3 = 24 + i;
    bool is_contact = contactSensorHandles_[i].isContact();

    scalar_t high_suspect_number(100);
    q.block(q_index, q_index, 3, 3) = (is_contact ? 1. : high_suspect_number) * q.block(q_index, q_index, 3, 3);
    r.block(r_index1, r_index1, 3, 3) = 1. * r.block(r_index1, r_index1, 3, 3);
    r.block(r_index2, r_index2, 3, 3) = (is_contact ? 1. : high_suspect_number) * r.block(r_index2, r_index2, 3, 3);
    r(r_index3, r_index3) = (is_contact ? 1. : high_suspect_number) * r(r_index3, r_index3);

    ps_.segment(3 * i, 3) = -1. * ee_pos[i];
    ps_.segment(3 * i, 3)[2] += foot_radius;
    vs_.segment(3 * i, 3) = -1. * ee_vel[i];
  }
  Eigen::Matrix<scalar_t, 3, 1> g(0, 0, -9.81);
  Eigen::Matrix<scalar_t, 3, 1> imu_accel(imuSensorHandle_.getLinearAcceleration()[0], imuSensorHandle_.getLinearAcceleration()[1],
                                          imuSensorHandle_.getLinearAcceleration()[2]);
  Eigen::Matrix<scalar_t, 3, 1> accel = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat)) * imu_accel + g;
  Eigen::Matrix<scalar_t, 4, 1> pzs = Eigen::Matrix<scalar_t, 4, 1>::Zero();

  Eigen::Matrix<scalar_t, 28, 1> y;
  y << ps_, vs_, pzs;
  xHat_ = a_ * xHat_ + b_ * accel;
  Eigen::Matrix<scalar_t, 18, 18> at = a_.transpose();
  Eigen::Matrix<scalar_t, 18, 18> pm = a_ * p_ * at + q;
  Eigen::Matrix<scalar_t, 18, 28> ct = c_.transpose();
  Eigen::Matrix<scalar_t, 28, 1> y_model = c_ * xHat_;
  Eigen::Matrix<scalar_t, 28, 1> ey = y - y_model;
  Eigen::Matrix<scalar_t, 28, 28> s = c_ * pm * ct + r;

  Eigen::Matrix<scalar_t, 28, 1> s_ey = s.lu().solve(ey);
  xHat_ += pm * ct * s_ey;

  Eigen::Matrix<scalar_t, 28, 18> s_c = s.lu().solve(c_);
  p_ = (Eigen::Matrix<scalar_t, 18, 18>::Identity() - pm * ct * s_c) * pm;

  Eigen::Matrix<scalar_t, 18, 18> pt = p_.transpose();
  p_ = (p_ + pt) / 2.0;

  if (p_.block(0, 0, 2, 2).determinant() > 0.000001) {
    p_.block(0, 2, 2, 16).setZero();
    p_.block(2, 0, 16, 2).setZero();
    p_.block(0, 0, 2, 2) /= 10.;
  }

  if (topicUpdated_) {
    updateFromTopic();
    topicUpdated_ = false;
  }

  updateLinear(xHat_.segment<3>(0), xHat_.segment<3>(3));

  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = xHat_.segment<3>(0)(0);
  odom.pose.pose.position.y = xHat_.segment<3>(0)(1);
  odom.pose.pose.position.z = xHat_.segment<3>(0)(2);
  odom.pose.pose.orientation.x = quat.x();
  odom.pose.pose.orientation.y = quat.y();
  odom.pose.pose.orientation.z = quat.z();
  odom.pose.pose.orientation.w = quat.w();
  odom.pose.pose.orientation.x = quat.x();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom.pose.covariance[i * 6 + j] = p_(i, j);
      odom.pose.covariance[6 * (3 + i) + (3 + j)] = imuSensorHandle_.getOrientationCovariance()[i * 3 + j];
    }
  }
  //  The twist in this message should be specified in the coordinate frame given by the child_frame_id: "base"
  vector_t twist = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat)).transpose() * xHat_.segment<3>(3);
  odom.twist.twist.linear.x = twist.x();
  odom.twist.twist.linear.y = twist.y();
  odom.twist.twist.linear.z = twist.z();
  odom.twist.twist.angular.x = angular_vel_local.x();
  odom.twist.twist.angular.y = angular_vel_local.y();
  odom.twist.twist.angular.z = angular_vel_local.z();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      odom.twist.covariance[i * 6 + j] = p_.block<3, 3>(3, 3)(i, j);
      odom.twist.covariance[6 * (3 + i) + (3 + j)] = imuSensorHandle_.getAngularVelocityCovariance()[i * 3 + j];
    }
  }
  publishMsgs(odom, time);

  return rbdState_;
}

void KalmanFilterEstimate::updateFromTopic() {
  auto* msg = buffer_.readFromRT();

  tf2::Transform world2sensor;
  world2sensor.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  world2sensor.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                                           msg->pose.pose.orientation.w));

  if (world2odom_.getRotation() == tf2::Quaternion::getIdentity())  // First received
  {
    tf2::Transform odom2sensor;
    try {
      geometry_msgs::TransformStamped tf_msg = tfBuffer_.lookupTransform("odom", msg->child_frame_id, msg->header.stamp);
      tf2::fromMsg(tf_msg.transform, odom2sensor);
    } catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      return;
    }
    world2odom_ = world2sensor * odom2sensor.inverse();
  }
  tf2::Transform base2sensor;
  try {
    geometry_msgs::TransformStamped tf_msg = tfBuffer_.lookupTransform("base", msg->child_frame_id, msg->header.stamp);
    tf2::fromMsg(tf_msg.transform, base2sensor);
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }
  tf2::Transform odom2base = world2odom_.inverse() * world2sensor * base2sensor.inverse();
  //  vector3_t new_pos(odom2base.getOrigin().x(), odom2base.getOrigin().y(), odom2base.getOrigin().z());
  vector3_t new_pos(odom2base.getOrigin().x(), odom2base.getOrigin().y(), xHat_(2));
  vector_t delta_pos = new_pos - xHat_.segment<3>(0);
  xHat_.segment<3>(0) = new_pos;
  for (size_t i = 0; i < 4; ++i) {
    xHat_.segment<3>(6 + i * 3) += delta_pos;
  }
}

void KalmanFilterEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) {
  buffer_.writeFromNonRT(*msg);
  topicUpdated_ = true;
}

}  // namespace legged
