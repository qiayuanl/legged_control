//
// Created by qiayuan on 23-1-29.
//

#pragma once

#include <ocs2_self_collision/SelfCollisionConstraint.h>

#include "legged_interface/LeggedRobotPreComputation.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class LeggedSelfCollisionConstraint final : public SelfCollisionConstraint {
 public:
  LeggedSelfCollisionConstraint(const CentroidalModelPinocchioMapping& mapping, PinocchioGeometryInterface pinocchioGeometryInterface,
                                scalar_t minimumDistance)
      : SelfCollisionConstraint(mapping, std::move(pinocchioGeometryInterface), minimumDistance) {}
  ~LeggedSelfCollisionConstraint() override = default;
  LeggedSelfCollisionConstraint(const LeggedSelfCollisionConstraint& other) = default;
  LeggedSelfCollisionConstraint* clone() const override { return new LeggedSelfCollisionConstraint(*this); }

  const PinocchioInterface& getPinocchioInterface(const PreComputation& preComputation) const override {
    return cast<LeggedRobotPreComputation>(preComputation).getPinocchioInterface();
  }
};

}  // namespace legged
