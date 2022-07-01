//
// Created by qiayuan on 2022/7/1.
//

#pragma once

#include "quad_wbc/ho_qp.h"

#include <ocs2_legged_robot/LeggedRobotInterface.h>

namespace quad_ros
{
using namespace legged_robot;

// Decision Variables: x = [\dot u^T, F^T, \tau^T]^T

class Wbc
{
public:
  Wbc(LeggedRobotInterface& legged_interface);
  vector_t update(const vector_t& rbd_state, const vector_t& input);

private:
  Task formulateFloatingBaseEomTask();
  Task formulateTorqueLimitsTask();
  Task formulateContactForceTask(const vector_t& input);
  Task formulateFrictionConeTask(const vector_t& input);

  size_t num_decision_vars_;
  PinocchioInterface& pino_interface_;
  const CentroidalModelInfo& info_;
};

}  // namespace quad_ros