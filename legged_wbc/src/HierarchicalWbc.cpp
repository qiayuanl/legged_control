//
// Created by skywoodsz on 2023/4/9.
//

#include "legged_wbc/HierarchicalWbc.h"

#include "legged_wbc/HoQp.h"

namespace legged {
vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period) {
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask()
          + formulateFrictionConeTask() + formulateNoContactMotionTask(); 
  Task task1 = formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask()
          + formulateBaseXYLinearAccelTask() + formulateSwingLegTask();
  Task task2 = formulateContactForceTask(inputDesired);
  HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

  vector_t x_optimal = hoQp.getSolutions();
  return WbcBase::updateCmd(x_optimal);
}

}  // namespace legged
