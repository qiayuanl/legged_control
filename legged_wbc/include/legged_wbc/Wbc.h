//
// Created by qiayuan on 2022/7/1.
//

#pragma once

#include "legged_wbc/HoQp.h"

#include <legged_interface/LeggedInterface.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

// Decision Variables: x = [\dot u^T, F^T, \tau^T]^T
class Wbc {
 public:
  Wbc(const std::string& taskFile, LeggedInterface& leggedInterface, const PinocchioEndEffectorKinematics& eeKinematics, bool verbose);
  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, vector_t& rbdStateMeasured, size_t mode);

 private:
  void loadTasksSetting(const std::string& taskFile, bool verbose);

  Task formulateFloatingBaseEomTask();
  Task formulateTorqueLimitsTask();
  Task formulateNoContactMotionTask();
  Task formulateFrictionConeTask();
  Task formulateBaseAccelTask();
  Task formulateSwingLegTask();
  Task formulateContactForceTask();

  size_t numDecisionVars_;
  PinocchioInterface& pinoInterface_;
  const CentroidalModelInfo& info_;
  PinocchioCentroidalDynamics centroidalDynamics_;
  CentroidalModelPinocchioMapping mapping_;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;

  vector_t stateDesired_, inputDesired_, qMeasured_, vMeasured_;
  matrix_t j_, dj_;
  contact_flag_t contactFlag_{};
  size_t numContacts_{};

  // Task Parameters:
  vector_t torqueLimits_;
  scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
};

}  // namespace legged
