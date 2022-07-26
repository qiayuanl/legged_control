//
// Created by qiayuan on 2022/7/1.
//
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_wbc/wbc.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

namespace legged
{
Wbc::Wbc(const std::string& task_file, LeggedInterface& legged_interface,
         const PinocchioEndEffectorKinematics& ee_kinematics, bool verbose)
  : pino_interface_(legged_interface.getPinocchioInterface())
  , info_(legged_interface.getCentroidalModelInfo())
  , centroidal_dynamics_(info_)
  , mapping_(info_)
  , ee_kinematics_(ee_kinematics.clone())
{
  num_decision_vars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
  centroidal_dynamics_.setPinocchioInterface(pino_interface_);
  mapping_.setPinocchioInterface(pino_interface_);
  measured_q_ = vector_t(info_.generalizedCoordinatesNum);
  measured_v_ = vector_t(info_.generalizedCoordinatesNum);
  ee_kinematics_->setPinocchioInterface(pino_interface_);

  loadTasksSetting(task_file, verbose);
}

vector_t Wbc::update(const vector_t& state_desired, const vector_t& input_desired, vector_t& measured_rbd_state,
                     size_t mode)
{
  state_desired_ = state_desired;
  input_desired_ = input_desired;

  contact_flag_ = modeNumber2StanceLeg(mode);
  num_contacts_ = 0;
  for (bool flag : contact_flag_)
    if (flag)
      num_contacts_++;

  measured_q_.head<3>() = measured_rbd_state.segment<3>(3);
  measured_q_.segment<3>(3) = measured_rbd_state.head<3>();
  measured_q_.tail(info_.actuatedDofNum) = measured_rbd_state.segment(6, info_.actuatedDofNum);

  measured_v_.head<3>() = measured_rbd_state.segment<3>(info_.generalizedCoordinatesNum + 3);
  measured_v_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      measured_q_.segment<3>(3), measured_rbd_state.segment<3>(info_.generalizedCoordinatesNum));
  measured_v_.tail(info_.actuatedDofNum) =
      measured_rbd_state.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

  const auto& model = pino_interface_.getModel();
  auto& data = pino_interface_.getData();

  // For floating base EoM task
  pinocchio::forwardKinematics(model, data, measured_q_, measured_v_);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::crba(model, data, measured_q_);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model, data, measured_q_, measured_v_);
  j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
  {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  // For not contact motion task
  pinocchio::computeJointJacobiansTimeVariation(model, data, measured_q_, measured_v_);
  dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
  {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i],
                                             pinocchio::LOCAL_WORLD_ALIGNED, jac);
    dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  // For base acceleration task
  updateCentroidalDynamics(pino_interface_, info_, measured_q_);

  Task task_0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() +
                formulateNoContactMotionTask();
  Task task_1 = formulateBaseAccelTask() + formulateSwingLegTask();
  Task task_2 = formulateContactForceTask();
  HoQp ho_qp(task_2, std::make_shared<HoQp>(task_1, std::make_shared<HoQp>(task_0)));

  return ho_qp.getSolutions();
}

Task Wbc::formulateFloatingBaseEomTask()
{
  auto& data = pino_interface_.getData();

  matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
  s.block(0, 0, info_.actuatedDofNum, 6).setZero();
  s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

  matrix_t a(info_.generalizedCoordinatesNum, num_decision_vars_);
  vector_t b(info_.generalizedCoordinatesNum);
  a << data.M, -j_.transpose(), -s.transpose();
  b = -data.nle;
  return Task(a, b, matrix_t(), vector_t());
}

Task Wbc::formulateTorqueLimitsTask()
{
  matrix_t d(2 * info_.actuatedDofNum, num_decision_vars_);
  d.setZero();
  matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
  d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
          info_.actuatedDofNum) = i;
  d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
          info_.actuatedDofNum) = -i;
  vector_t f(2 * info_.actuatedDofNum);
  for (size_t l = 0; l < 2 * info_.actuatedDofNum / 3; ++l)
    f.segment<3>(3 * l) = torque_limits_;

  return Task(matrix_t(), vector_t(), d, f);
}

Task Wbc::formulateNoContactMotionTask()
{
  matrix_t a(3 * num_contacts_, num_decision_vars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; i++)
    if (contact_flag_[i])
    {
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * measured_v_;
      j++;
    }

  return Task(a, b, matrix_t(), matrix_t());
}

Task Wbc::formulateFrictionConeTask()
{
  matrix_t a(3 * (info_.numThreeDofContacts - num_contacts_), num_decision_vars_);
  a.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    if (!contact_flag_[i])
      a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
  vector_t b(a.rows());
  b.setZero();

  matrix_t friction_pyramic(5, 3);
  friction_pyramic << 0, 0, -1, 1, 0, -friction_coeff_, -1, 0, -friction_coeff_, 0, 1, -friction_coeff_, 0, -1,
      -friction_coeff_;

  matrix_t d(5 * num_contacts_ + 3 * (info_.numThreeDofContacts - num_contacts_), num_decision_vars_);
  d.setZero();
  j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    if (contact_flag_[i])
      d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = friction_pyramic;
  vector_t f = Eigen::VectorXd::Zero(d.rows());

  return Task(a, b, d, f);
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

  const vector3_t angular_velocity = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
      measured_q_.segment<3>(3), measured_v_.segment<3>(3));
  b.segment<3>(3) -= a_base_inv.block<3, 3>(3, 3) * angular_velocity.cross(a_base.block<3, 3>(3, 3) * angular_velocity);

  return Task(a, b, matrix_t(), vector_t());
}

Task Wbc::formulateSwingLegTask()
{
  std::vector<vector3_t> pos_measured = ee_kinematics_->getPosition(vector_t());
  std::vector<vector3_t> vel_measured = ee_kinematics_->getVelocity(vector_t(), vector_t());
  vector_t q_desired = mapping_.getPinocchioJointPosition(state_desired_);
  vector_t v_desired = mapping_.getPinocchioJointVelocity(state_desired_, input_desired_);
  const auto& model = pino_interface_.getModel();
  auto& data = pino_interface_.getData();
  pinocchio::forwardKinematics(model, data, q_desired, v_desired);
  pinocchio::updateFramePlacements(model, data);
  std::vector<vector3_t> pos_desired = ee_kinematics_->getPosition(vector_t());
  std::vector<vector3_t> vel_desired = ee_kinematics_->getVelocity(vector_t(), vector_t());

  matrix_t a(3 * (info_.numThreeDofContacts - num_contacts_), num_decision_vars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    if (!contact_flag_[i])
    {
      vector3_t accel = swing_kp_ * (pos_desired[i] - pos_measured[i]) + swing_kd_ * (vel_desired[i] - vel_measured[i]);
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * measured_v_;
      j++;
    }

  return Task(a, b, matrix_t(), vector_t());
}

Task Wbc::formulateContactForceTask()
{
  matrix_t a(3 * info_.numThreeDofContacts, num_decision_vars_);
  vector_t b(a.rows());
  a.setZero();

  for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
  b = input_desired_.head(a.rows());

  return Task(a, b, matrix_t(), vector_t());
}

void Wbc::loadTasksSetting(const std::string& task_file, bool verbose)
{
  // Load task file
  torque_limits_ = vector_t(info_.actuatedDofNum / 4);
  loadData::loadEigenMatrix(task_file, "torqueLimitsTask", torque_limits_);
  if (verbose)
  {
    std::cerr << "\n #### Torque Limits Task: \n";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "\n #### HAA HFE KFE: " << torque_limits_.transpose() << "\n";
    std::cerr << " #### =============================================================================\n";
  }
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(task_file, pt);
  std::string prefix = "frictionConeTask.";
  if (verbose)
  {
    std::cerr << "\n #### Friction Cone Task: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, friction_coeff_, prefix + "frictionCoefficient", verbose);
  if (verbose)
  {
    std::cerr << " #### =============================================================================\n";
  }
  prefix = "swingLegTask.";
  if (verbose)
  {
    std::cerr << "\n #### Swing Leg Task: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, swing_kp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, swing_kd_, prefix + "kd", verbose);
}

}  // namespace legged
