//
// Created by qiayuan on 2022/7/1.
//
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "quad_wbc/wbc.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

namespace quad_ros
{
Wbc::Wbc(LeggedRobotInterface& legged_interface)
  : pino_interface_(legged_interface.getPinocchioInterface())
  , info_(legged_interface.getCentroidalModelInfo())
  , centroidal_dynamics_(info_)
{
  num_decision_vars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
  centroidal_dynamics_.setPinocchioInterface(pino_interface_);
}

vector_t Wbc::update(const vector_t& state_desired, const vector_t& input_desired, vector_t& measured_rbd_state,
                     size_t mode)
{
  state_desired_ = state_desired;
  input_desired_ = input_desired;
  measured_rbd_state_ = measured_rbd_state;

  contact_flag_ = modeNumber2StanceLeg(mode);
  num_contacts_ = 0;
  for (bool flag : contact_flag_)
    if (flag)
      num_contacts_++;

  vector_t q_pino(info_.generalizedCoordinatesNum), v_pino(info_.generalizedCoordinatesNum);

  q_pino.segment<3>(0) = measured_rbd_state.segment<3>(3);
  q_pino.segment<3>(3) = measured_rbd_state.head<3>();
  q_pino.tail(info_.actuatedDofNum) = measured_rbd_state.segment(6, info_.actuatedDofNum);

  v_pino.segment<3>(0) = measured_rbd_state.segment<3>(info_.generalizedCoordinatesNum + 3);
  v_pino.segment<3>(3) = measured_rbd_state.segment<3>(info_.generalizedCoordinatesNum);
  v_pino.tail(info_.actuatedDofNum) =
      measured_rbd_state.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum);

  const auto& model = pino_interface_.getModel();
  auto& data = pino_interface_.getData();

  pinocchio::forwardKinematics(model, data, q_pino, v_pino);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::crba(model, data, q_pino);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model, data, q_pino, v_pino);

  updateCentroidalDynamics(pino_interface_, info_, q_pino);

  Task task_0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask();
  Task task_1 = formulateBaseAccelTask();
  Task task_2 = formulateContactForceTask();
  HoQp ho_qp(task_1, std::make_shared<HoQp>(task_2, std::make_shared<HoQp>(task_0)));

  return ho_qp.getSolutions();
}

Task Wbc::formulateFloatingBaseEomTask()
{
  const auto& model = pino_interface_.getModel();
  auto& data = pino_interface_.getData();

  matrix_t j(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
  {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    j.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }
  matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
  s.block(0, 0, info_.actuatedDofNum, info_.generalizedCoordinatesNum - info_.actuatedDofNum).setZero();
  s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

  matrix_t a(info_.generalizedCoordinatesNum, num_decision_vars_);
  vector_t b(info_.generalizedCoordinatesNum);
  a << data.M, -j.transpose(), -s.transpose();
  b = -data.nle;
  return Task(a, b, matrix_t(), vector_t());
}

Task Wbc::formulateTorqueLimitsTask()
{
  scalar_t torque_max = 40;
  matrix_t d(2 * info_.actuatedDofNum, num_decision_vars_);
  d.setZero();
  matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
  d.block(0, info_.generalizedCoordinatesNum + info_.numThreeDofContacts, info_.actuatedDofNum, info_.actuatedDofNum) =
      i;
  d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + info_.numThreeDofContacts, info_.actuatedDofNum,
          info_.actuatedDofNum) = -i;
  vector_t f(2 * info_.actuatedDofNum);
  f.setConstant(torque_max);
  return Task(matrix_t(), vector_t(), d, f);
}

Task Wbc::formulateNoContactMotionTask()
{
  return Task();
}

Task Wbc::formulateFrictionConeTask()
{
  scalar_t friction_coeff = 0.5;
  matrix_t friction_pyramic(5, 3);
  friction_pyramic << 0, 0, -1, 1, 0, -friction_coeff, -1, 0, -friction_coeff, 0, 1, -friction_coeff, 0, -1,
      -friction_coeff;

  matrix_t d(5 * num_contacts_, num_decision_vars_);
  d.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    if (contact_flag_[i])
      d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = friction_pyramic;

  vector_t f = Eigen::VectorXd::Zero(5 * num_contacts_);

  return Task(matrix_t(), vector_t(), d, f);
}

Task Wbc::formulateBaseAccelTask()
{
  matrix_t a(6, num_decision_vars_);
  a.setZero();
  a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

  const vector_t momentum_rate =
      info_.robotMass * centroidal_dynamics_.getValue(0, state_desired_, input_desired_).head(6);
  const Eigen::Matrix<scalar_t, 6, 6> a_base = getCentroidalMomentumMatrix(pino_interface_).template leftCols<6>();
  const auto a_base_inv = computeFloatingBaseCentroidalMomentumMatrixInverse(a_base);
  vector_t b = a_base_inv * momentum_rate;
  return Task(a, b, matrix_t(), vector_t());
}

Task Wbc::formulateSwingLegTask()
{
  vector_t e_pos =
      centroidal_model::getJointAngles(state_desired_, info_) - measured_rbd_state_.segment(6, info_.actuatedDofNum);
  vector_t e_vel = centroidal_model::getJointVelocities(input_desired_, info_) -
                   measured_rbd_state_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum);
  vector_t acc = 350 * e_pos + 37 * e_vel;
  size_t dim = 3 * (info_.numThreeDofContacts - num_contacts_);

  matrix_t a(dim, num_decision_vars_);
  vector_t b(dim);
  a.setZero();
  size_t j = 0;
  size_t k[4] = { 0, 2, 1, 3 };
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    if (!contact_flag_[i])
    {
      a.block(3 * j, 6 + 3 * k[i], 3, 3) = matrix_t::Identity(3, 3);
      b.segment(3 * j, 3) = acc.segment<3>(3 * k[i]);
      ++j;
    }
  return Task(a, b, matrix_t(), vector_t());
}

Task Wbc::formulateContactForceTask()
{
  size_t dim = 3 * info_.numThreeDofContacts;

  matrix_t a(dim, num_decision_vars_);
  vector_t b(dim);
  a.setZero();
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
  b = input_desired_.head(dim);

  return Task(a, b, matrix_t(), vector_t());
}

}  // namespace quad_ros
