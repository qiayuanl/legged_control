//
// Created by skywoodsz on 2023/4/9.
//
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_wbc/WbcBase.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>

namespace legged {
WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics)
    : pinocchioInterfaceMeasured_(pinocchioInterface),
      pinocchioInterfaceDesired_(pinocchioInterface),
      info_(std::move(info)),
      mapping_(info_),
      inputLast_(vector_t::Zero(info_.inputDim)),
      eeKinematics_(eeKinematics.clone()) {
  numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts;
  qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  qDesired_ = vector_t(info_.generalizedCoordinatesNum);
  vDesired_ = vector_t(info_.generalizedCoordinatesNum);
  baseAccDesired_ = vector_t(6);
  base_j_ = matrix_t(6, info_.generalizedCoordinatesNum);
  base_dj_ = matrix_t(6, info_.generalizedCoordinatesNum);
}

vector_t WbcBase::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                         scalar_t period) {
  contactFlag_ = modeNumber2StanceLeg(mode);
  numContacts_ = 0;
  for (bool flag : contactFlag_) {
    if (flag) {
      numContacts_++;
    }
  }

  updateMeasured(rbdStateMeasured);
  updateDesired(stateDesired, inputDesired, period);

  return {};
}

void WbcBase::updateMeasured(const vector_t& rbdStateMeasured) {
  qMeasured_.setZero();
  vMeasured_.setZero();

  qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
  qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
  qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
  vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
  vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
  vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

  const auto& model = pinocchioInterfaceMeasured_.getModel();
  auto& data = pinocchioInterfaceMeasured_.getData();

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

  // For base motion tracking task
  Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> base_j, base_dj;
  base_j.setZero(6, info_.generalizedCoordinatesNum);
  base_dj.setZero(6, info_.generalizedCoordinatesNum);
  pinocchio::getFrameJacobian(model, data, model.getBodyId("base"), pinocchio::LOCAL_WORLD_ALIGNED, base_j);
  pinocchio::getFrameJacobianTimeVariation(model, data, model.getBodyId("base"), pinocchio::LOCAL_WORLD_ALIGNED, base_dj);
  base_j_.setZero(); base_j_ = base_j;
  base_dj_.setZero(); base_dj_ = base_dj;
}

void WbcBase::updateDesired(const ocs2::vector_t &stateDesired, const ocs2::vector_t &inputDesired, ocs2::scalar_t period) {
    const auto& model = pinocchioInterfaceDesired_.getModel();
    auto& data = pinocchioInterfaceDesired_.getData();

    qDesired_.setZero();
    vDesired_.setZero();

    mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
    qDesired_ = mapping_.getPinocchioJointPosition(stateDesired);
    pinocchio::forwardKinematics(model, data, qDesired_);
    pinocchio::computeJointJacobians(model, data, qDesired_);
    pinocchio::updateFramePlacements(model, data);
    updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired_);

    vDesired_ = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

    pinocchio::forwardKinematics(model, data, qDesired_, vDesired_);

    // update base acc desired
    vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
    inputLast_ = inputDesired;

    const auto& A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);
    const Matrix6 Ab = A.template leftCols<6>();
    const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
    auto Aj = A.rightCols(info_.actuatedDofNum);
    const auto ADot = pinocchio::dccrba(model, data, qDesired_, vDesired_);
    Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
    centroidalMomentumRate.noalias() -= ADot * vDesired_;
    centroidalMomentumRate.noalias() -= Aj * jointAccel;

    baseAccDesired_.setZero();
    baseAccDesired_ = AbInv * centroidalMomentumRate;
}

// EoM
// [Mb, -J^Tb]x = -hb
Task WbcBase::formulateFloatingBaseEomTask() {
    matrix_t a(6, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();

    auto& data = pinocchioInterfaceMeasured_.getData();

    matrix_t Mb, Jb_T;
    vector_t hb;
    Mb = data.M.topRows(6);
    hb = data.nle.topRows(6);
    Jb_T = j_.transpose().topRows(6);

    a << Mb, -Jb_T;
    b = -hb;

    return {a, b, matrix_t(), matrix_t()};
}

// torque limit
// tau_min - hj <= [Mj, -Jj^T] <= tau_max - hj
Task WbcBase::formulateTorqueLimitsTask() {
    matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
    vector_t f(d.rows());
    d.setZero();
    f.setZero();

    auto& data = pinocchioInterfaceMeasured_.getData();

    matrix_t Mj, Jj_T;
    vector_t hj;
    Mj = data.M.bottomRows(info_.actuatedDofNum);
    Jj_T = j_.transpose().bottomRows(info_.actuatedDofNum);
    hj = data.nle.bottomRows(info_.actuatedDofNum);

    d.block(0, 0, info_.actuatedDofNum, numDecisionVars_) << Mj, -Jj_T;
    d.block(info_.actuatedDofNum, 0, info_.actuatedDofNum, numDecisionVars_) << -Mj, Jj_T;

    for (size_t l = 0; l < 2 * info_.actuatedDofNum / 3; ++l) {
        f.segment<3>(3 * l) = torqueLimits_;
    }

    f.segment<12>(0) -= hj;
    f.segment<12>(info_.actuatedDofNum) += hj;

    return {matrix_t(), vector_t(), d, f};
}

// [J, 0] x = -\dot J v
Task WbcBase::formulateNoContactMotionTask() {
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

    return {a, b, matrix_t(), vector_t()};
}

// no contact:
// [0, I] x = 0
// contact:
// [0, C] x <= 0
Task WbcBase::formulateFrictionConeTask() {
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

    matrix_t frictionPyramic(5, 3);  // clang-format off
    frictionPyramic << 0, 0, -1,
            1, 0, -frictionCoeff_,
            -1, 0, -frictionCoeff_,
            0, 1, -frictionCoeff_,
            0,-1, -frictionCoeff_;  // clang-format on

    matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
    d.setZero();
    j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        if (contactFlag_[i]) {
            d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
        }
    }
    vector_t f = Eigen::VectorXd::Zero(d.rows());

    return {a, b, d, f};
}

// Tracking base height motion task
Task WbcBase::formulateBaseHeightMotionTask() {
    matrix_t a(1, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();
    a.block(0, 2, 1, 1) = matrix_t::Identity(1, 1);

    b[0] = baseAccDesired_[2] + baseHeightKp_ * (qDesired_[2] - qMeasured_[2])
           + baseHeightKd_ * (vDesired_[2] - vMeasured_[2]);

    return {a, b, matrix_t(), vector_t()};
}

// Tracking base angular motion task
Task WbcBase::formulateBaseAngularMotionTask(){
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 3, info_.generalizedCoordinatesNum) = base_j_.block(3, 0, 3, info_.generalizedCoordinatesNum);

    vector3_t eulerAngles = qMeasured_.segment<3>(3);

    // from derivative euler to angular
    vector3_t vMeasuredGlobal =
            getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles, vMeasured_.segment<3>(3));
    vector3_t vDesiredGlobal =
            getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles, vDesired_.segment<3>(3));

    // from euler to rotation
    vector3_t eulerAnglesDesired;
    eulerAnglesDesired << qDesired_.segment<3>(3);
    matrix3_t rotationBaseMeasuredToWorld =
            getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAngles);
    matrix3_t rotationBaseReferenceToWorld =
            getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAnglesDesired);

    vector3_t error = rotationErrorInWorld<scalar_t>(rotationBaseReferenceToWorld, rotationBaseMeasuredToWorld);

    // desired acc
    vector3_t accDesired = getGlobalAngularAccelerationFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles,
                                                           vDesired_.segment<3>(3), baseAccDesired_.segment<3>(3));

    b = accDesired + baseAngularKp_  * error + baseAngularKd_ * (vDesiredGlobal - vMeasuredGlobal)
        - base_dj_.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

    return {a, b, matrix_t(), vector_t()};
}

// Tracking base xy linear motion task
Task WbcBase::formulateBaseXYLinearAccelTask() {
    matrix_t a(2, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 2, 2) = matrix_t::Identity(2, 2);

    b = baseAccDesired_.segment<2>(0);

    return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateSwingLegTask() {
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
  std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
  std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

Task WbcBase::formulateContactForceTask(const vector_t& inputDesired) const {
  matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();

  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
  }
  b = inputDesired.head(a.rows());

  return {a, b, matrix_t(), vector_t()};
}

// return [\dot u^T, F^T, \tau^T]^T
vector_t WbcBase::updateCmd(ocs2::vector_t x_optimal) {
    auto& data = pinocchioInterfaceMeasured_.getData();

    matrix_t Mj, Jj_T;
    vector_t hj;
    Mj = data.M.bottomRows(info_.actuatedDofNum);
    Jj_T = j_.transpose().bottomRows(info_.actuatedDofNum);
    hj = data.nle.bottomRows(info_.actuatedDofNum);
    matrix_t a = (matrix_t(info_.actuatedDofNum, getNumDecisionVars())<< Mj, -Jj_T).finished();

    vector_t torque_optimal = a * x_optimal + hj;

    vector_t cmd = (vector_t(numDecisionVars_ + info_.actuatedDofNum)<<x_optimal, torque_optimal).finished();

    return cmd;
}

void WbcBase::loadTasksSetting(const std::string& taskFile, bool verbose) {
  // Load task file
  torqueLimits_ = vector_t(info_.actuatedDofNum / 4);
  loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimits_);
  if (verbose) {
    std::cerr << "\n #### Torque Limits Task:";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "\n #### HAA HFE KFE: " << torqueLimits_.transpose() << "\n";
    std::cerr << " #### =============================================================================\n";
  }
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "frictionConeTask.";
  if (verbose) {
    std::cerr << "\n #### Friction Cone Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }
  prefix = "swingLegTask.";
  if (verbose) {
    std::cerr << "\n #### Swing Leg Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, swingKp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, swingKd_, prefix + "kd", verbose);
  prefix = "baseHeightTask.";
  if (verbose) {
      std::cerr << "\n #### Base Height Task:";
      std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, baseHeightKp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, baseHeightKd_, prefix + "kd", verbose);
  prefix = "baseAngularTask.";
  if (verbose) {
      std::cerr << "\n #### Base Angular Task:";
      std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, baseAngularKp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, baseAngularKd_, prefix + "kd", verbose);
}

}  // namespace legged
