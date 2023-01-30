//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WeightedWbc.h"

#include <eiquadprog/eiquadprog-fast.hpp>

namespace legged {

vector_t WeightedWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                             scalar_t period) {
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  Task constraints =
      formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
  Task weighedTask = formulateSwingLegTask() * weightSwingLeg_ +
                     formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
                     formulateContactForceTask(inputDesired) * weightContactForce_;

  matrix_t H = weighedTask.a_.transpose() * weighedTask.a_;
  vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;
  H.diagonal().array() += 1e-20;

  eiquadprog::solvers::EiquadprogFast qp;
  qp.reset(getNumDecisionVars(), constraints.b_.size(), constraints.f_.size());
  vector_t qpSol(getNumDecisionVars());
  qp.solve_quadprog(H, g, constraints.a_, -constraints.b_, -constraints.d_, constraints.f_, qpSol);
  return qpSol;
}

void WeightedWbc::loadTasksSetting(const std::string& taskFile, bool verbose) {
  WbcBase::loadTasksSetting(taskFile, verbose);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "weight.";
  if (verbose) {
    std::cerr << "\n #### WBC weight:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
  loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
  loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
}

}  // namespace legged
