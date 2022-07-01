//
// Created by qiayuan on 2022/7/1.
//
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "quad_wbc/wbc.h"

#include <pinocchio/algorithm/frames.hpp>

namespace quad_ros
{
Wbc::Wbc(LeggedRobotInterface& legged_interface)
  : legged_interface_(legged_interface)
  , pino_interface_(legged_interface.getPinocchioInterface())
  , info_(legged_interface.getCentroidalModelInfo())
{
  num_decision_vars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
}

void Wbc::update(const vector_t& rbd_state, const vector_t& input)
{
  vector_t q_pino(info_.generalizedCoordinatesNum);
  q_pino.setZero();

  q_pino.segment<3>(0) = rbd_state.segment<3>(3);
  q_pino.segment<3>(3) = rbd_state.head<3>();
  q_pino.tail(info_.actuatedDofNum) = rbd_state.segment(6, info_.actuatedDofNum);

  //  vector_t v_pino(info_.generalizedCoordinatesNum);
  //  v_pino.setZero();
  //  v_pino.segment<3>(0) = rbd_state.segment<3>(info_.generalizedCoordinatesNum + 3);
  //  v_pino.segment<3>(3) = rbd_state.segment<3>(info_.generalizedCoordinatesNum);
  //  v_pino.tail(info_.actuatedDofNum) = rbd_state.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum);

  const auto& model = pino_interface_.getModel();
  auto& data = pino_interface_.getData();

  pinocchio::forwardKinematics(model, data, q_pino);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
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
  scalar_t torque_max = 50;
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

Task Wbc::formulateContactForceTask()
{
  size_t dim = 3 * info_.numThreeDofContacts;

  matrix_t a(dim, num_decision_vars_);
  vector_t b(dim);
  a.setZero();
  a.block(0, info_.generalizedCoordinatesNum, dim, dim) = matrix_t::Identity(dim, dim);
  b = input_.head(dim);
  return Task(a, b, matrix_t(), vector_t());
}

}  // namespace quad_ros
