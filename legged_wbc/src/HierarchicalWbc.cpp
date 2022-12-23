//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/HierarchicalWbc.h"

#include "legged_wbc/HoQp.h"

namespace legged {
vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, vector_t& rbdStateMeasured, size_t mode) {
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode);

  Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
  Task task1 = formulateBaseAccelTask() + formulateSwingLegTask();
  Task task2 = formulateContactForceTask();
  HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

  return hoQp.getSolutions();
}

}  // namespace legged
