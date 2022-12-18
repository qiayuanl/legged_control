//
// Created by qiayuan on 2022/7/1.
//
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_wbc/Wbc.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace legged {
Wbc::Wbc(const std::string& taskFile, LeggedInterface& leggedInterface, const PinocchioEndEffectorKinematics& eeKinematics, bool verbose)
    : pinoInterface_(leggedInterface.getPinocchioInterface()),
      info_(leggedInterface.getCentroidalModelInfo()),
      centroidalDynamics_(info_),
      mapping_(info_),
      eeKinematics_(eeKinematics.clone()) {
  numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
  centroidalDynamics_.setPinocchioInterface(pinoInterface_);
  mapping_.setPinocchioInterface(pinoInterface_);
  qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  eeKinematics_->setPinocchioInterface(pinoInterface_);

  loadTasksSetting(taskFile, verbose);
}

vector_t Wbc::update(const vector_t& stateDesired, const vector_t& inputDesired, vector_t& rbdStateMeasured, size_t mode) {
  stateDesired_ = stateDesired;
  inputDesired_ = inputDesired;

  contactFlag_ = modeNumber2StanceLeg(mode);
  numContacts_ = 0;
  for (bool flag : contactFlag_) {
    if (flag) {
      numContacts_++;
    }
  }

  qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
  qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
  qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);

  vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
  vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
  vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

  const auto& model = pinoInterface_.getModel();
  auto& data = pinoInterface_.getData();

  // For floating base EoM task
  pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::crba(model, data, qMeasured_);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
  j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  // For not contact motion task
  pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
  dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  // For base acceleration task
  updateCentroidalDynamics(pinoInterface_, info_, qMeasured_);

  Task task_0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
  Task task_1 = formulateBaseAccelTask() + formulateSwingLegTask();
  Task task_2 = formulateContactForceTask();
  HoQp ho_qp(task_2, std::make_shared<HoQp>(task_1, std::make_shared<HoQp>(task_0)));

  return ho_qp.getSolutions();
}

Task Wbc::formulateFloatingBaseEomTask() {
  auto& data = pinoInterface_.getData();

  matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
  s.block(0, 0, info_.actuatedDofNum, 6).setZero();
  s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

  matrix_t a(info_.generalizedCoordinatesNum, numDecisionVars_);
  vector_t b(info_.generalizedCoordinatesNum);
  a << data.M, -j_.transpose(), -s.transpose();
  b = -data.nle;
  return Task(a, b, matrix_t(), vector_t());
}

Task Wbc::formulateTorqueLimitsTask() {
  matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
  d.setZero();
  matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
  d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum, info_.actuatedDofNum) = i;
  d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
          info_.actuatedDofNum) = -i;
  vector_t f(2 * info_.actuatedDofNum);
  for (size_t l = 0; l < 2 * info_.actuatedDofNum / 3; ++l) {
    f.segment<3>(3 * l) = torqueLimits_;
  }

  return Task(matrix_t(), vector_t(), d, f);
}

Task Wbc::formulateNoContactMotionTask() {
  matrix_t a(3 * numContacts_, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
    if (contactFlag_[i]) {
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return Task(a, b, matrix_t(), vector_t());
}

Task Wbc::formulateFrictionConeTask() {
  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  a.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
    }
  }
  vector_t b(a.rows());
  b.setZero();

  matrix_t friction_pyramic(5, 3);
  friction_pyramic << 0, 0, -1, 1, 0, -frictionCoeff_, -1, 0, -frictionCoeff_, 0, 1, -frictionCoeff_, 0, -1, -frictionCoeff_;

  matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  d.setZero();
  j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (contactFlag_[i]) {
      d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = friction_pyramic;
    }
  }
  vector_t f = Eigen::VectorXd::Zero(d.rows());

  return Task(a, b, d, f);
}

Task Wbc::formulateBaseAccelTask() {
  matrix_t a(6, numDecisionVars_);
  a.setZero();
  a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

  const vector_t momentum_rate = info_.robotMass * centroidalDynamics_.getValue(0, stateDesired_, inputDesired_).head(6);
  const Eigen::Matrix<scalar_t, 6, 6> a_base = getCentroidalMomentumMatrix(pinoInterface_).template leftCols<6>();
  const auto a_base_inv = computeFloatingBaseCentroidalMomentumMatrixInverse(a_base);
  vector_t b = a_base_inv * momentum_rate;

  const vector3_t angular_velocity =
      getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(qMeasured_.segment<3>(3), vMeasured_.segment<3>(3));
  b.segment<3>(3) -= a_base_inv.block<3, 3>(3, 3) * angular_velocity.cross(a_base.block<3, 3>(3, 3) * angular_velocity);

  return Task(a, b, matrix_t(), vector_t());
}

Task Wbc::formulateSwingLegTask() {
  std::vector<vector3_t> pos_measured = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> vel_measured = eeKinematics_->getVelocity(vector_t(), vector_t());
  vector_t q_desired = mapping_.getPinocchioJointPosition(stateDesired_);
  vector_t v_desired = mapping_.getPinocchioJointVelocity(stateDesired_, inputDesired_);
  const auto& model = pinoInterface_.getModel();
  auto& data = pinoInterface_.getData();
  pinocchio::forwardKinematics(model, data, q_desired, v_desired);
  pinocchio::updateFramePlacements(model, data);
  std::vector<vector3_t> pos_desired = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> vel_desired = eeKinematics_->getVelocity(vector_t(), vector_t());

  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      vector3_t accel = swingKp_ * (pos_desired[i] - pos_measured[i]) + swingKd_ * (vel_desired[i] - vel_measured[i]);
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return Task(a, b, matrix_t(), vector_t());
}

Task Wbc::formulateContactForceTask() {
  matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();

  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
  }
  b = inputDesired_.head(a.rows());

  return {a, b, matrix_t(), vector_t()};
}

void Wbc::loadTasksSetting(const std::string& taskFile, bool verbose) {
  // Load task file
  torqueLimits_ = vector_t(info_.actuatedDofNum / 4);
  loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimits_);
  if (verbose) {
    std::cerr << "\n #### Torque Limits Task: \n";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "\n #### HAA HFE KFE: " << torqueLimits_.transpose() << "\n";
    std::cerr << " #### =============================================================================\n";
  }
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "frictionConeTask.";
  if (verbose) {
    std::cerr << "\n #### Friction Cone Task: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }
  prefix = "swingLegTask.";
  if (verbose) {
    std::cerr << "\n #### Swing Leg Task: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, swingKp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, swingKd_, prefix + "kd", verbose);
}

}  // namespace legged
