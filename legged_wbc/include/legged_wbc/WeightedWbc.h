//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WbcBase.h"

namespace legged {

class WeightedWbc : public WbcBase {
 public:
  using WbcBase::WbcBase;

  vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, vector_t& rbdStateMeasured, size_t mode) override;

  void loadTasksSetting(const std::string& taskFile, bool verbose) override;

 protected:
 private:
  scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;
};

}  // namespace legged