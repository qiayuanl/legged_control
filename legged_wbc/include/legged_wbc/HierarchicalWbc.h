//
// Created by qiayuan on 22-12-23.
//
#pragma once

#include "legged_wbc/WbcBase.h"

namespace legged {

class HierarchicalWbc : public WbcBase {
 public:
  using WbcBase::WbcBase;

  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                  scalar_t period) override;
};

}  // namespace legged
