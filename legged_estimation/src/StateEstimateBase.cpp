//
// Created by qiayuan on 2021/11/15.
//

#include "legged_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

namespace legged {
using namespace legged_robot;

StateEstimateBase::StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                     const PinocchioEndEffectorKinematics& eeKinematics)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      info_(std::move(info)),
      eeKinematics_(eeKinematics.clone()),
      rbdState_(vector_t ::Zero(2 * info_.generalizedCoordinatesNum)) {
  ros::NodeHandle nh;
  odomPub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh, "odom", 10));

  posePub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>(nh, "pose", 10));
}

void StateEstimateBase::updateJointStates(const vector_t& jointPos, const vector_t& jointVel) {
  rbdState_.segment(6, info_.actuatedDofNum) = jointPos;
  rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = jointVel;
}

void StateEstimateBase::updateImu(const Eigen::Quaternion<scalar_t>& quat, const vector3_t& angularVelLocal,
                                  const vector3_t& linearAccelLocal, const matrix3_t& orientationCovariance,
                                  const matrix3_t& angularVelCovariance, const matrix3_t& linearAccelCovariance) {
  quat_ = quat;
  angularVelLocal_ = angularVelLocal;
  linearAccelLocal_ = linearAccelLocal;
  orientationCovariance_ = orientationCovariance;
  angularVelCovariance_ = angularVelCovariance;
  linearAccelCovariance_ = linearAccelCovariance;

  vector3_t zyx = quatToZyx(quat) - zyxOffset_;
  vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
      zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat), angularVelLocal));
  updateAngular(zyx, angularVelGlobal);
}

void StateEstimateBase::updateAngular(const vector3_t& zyx, const vector_t& angularVel) {
  rbdState_.segment<3>(0) = zyx;
  rbdState_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
}

void StateEstimateBase::updateLinear(const vector_t& pos, const vector_t& linearVel) {
  rbdState_.segment<3>(3) = pos;
  rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
}

void StateEstimateBase::publishMsgs(const nav_msgs::Odometry& odom) {
  ros::Time time = odom.header.stamp;
  scalar_t publishRate = 200;
  if (lastPub_ + ros::Duration(1. / publishRate) < time) {
    lastPub_ = time;
    if (odomPub_->trylock()) {
      odomPub_->msg_ = odom;
      odomPub_->unlockAndPublish();
    }
    if (posePub_->trylock()) {
      posePub_->msg_.header = odom.header;
      posePub_->msg_.pose = odom.pose;
      posePub_->unlockAndPublish();
    }
  }
}

}  // namespace legged
