//
// Created by skywoodsz on 2023/3/12.
//

#pragma once

#include "legged_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

// Decision Variables: x = [\dot u^T, F^T]^T
class WbcBase {
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

 public:
  WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

  virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                          scalar_t period);

 protected:
  void updateMeasured(const vector_t& rbdStateMeasured);
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired, ocs2::scalar_t period);
  vector_t updateCmd(vector_t x_optimal);

  size_t getNumDecisionVars() const { return numDecisionVars_; }

  Task formulateFloatingBaseEomTask();
  Task formulateTorqueLimitsTask();
  Task formulateNoContactMotionTask();
  Task formulateFrictionConeTask();
  Task formulateBaseHeightMotionTask();
  Task formulateBaseAngularMotionTask();
  Task formulateBaseXYLinearAccelTask();
  Task formulateSwingLegTask();
  Task formulateContactForceTask(const vector_t& inputDesired) const;


  size_t numDecisionVars_;
  PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  CentroidalModelInfo info_;

  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  CentroidalModelPinocchioMapping mapping_;

  vector_t qMeasured_, vMeasured_, inputLast_;
  vector_t qDesired_, vDesired_, baseAccDesired_;
  matrix_t j_, dj_;
  matrix_t base_j_, base_dj_;
  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // Task Parameters:
  vector_t torqueLimits_;
  scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
  scalar_t baseHeightKp_{}, baseHeightKd_{};
  scalar_t baseAngularKp_{}, baseAngularKd_{};
};

}  // namespace legged
