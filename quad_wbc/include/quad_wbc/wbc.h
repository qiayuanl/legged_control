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
  void update(const vector_t& rbd_state, const vector_t& input);

private:
  Task formulateFloatingBaseEomTask();
  Task formulateTorqueLimitsTask();
  Task formulateContactForceTask();

  size_t num_decision_vars_;
  vector_t input_;
  LeggedRobotInterface& legged_interface_;
  PinocchioInterface& pino_interface_;
  const CentroidalModelInfo& info_;
};

}  // namespace quad_ros