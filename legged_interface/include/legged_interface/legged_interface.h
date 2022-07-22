//
// Created by qiayuan on 2022/7/16.
//

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_sqp/MultipleShootingSettings.h>

#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/initialization/LeggedRobotInitializer.h>
#include <ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h>

namespace legged
{
using namespace ocs2;
using namespace legged_robot;

class LeggedInterface : public RobotInterface
{
public:
  LeggedInterface(const std::string& task_file, const std::string& urdf_file, const std::string& reference_file,
                  bool verbose);

  ~LeggedInterface() override = default;

  virtual void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                          const std::string& referenceFile, bool verbose);

  const OptimalControlProblem& getOptimalControlProblem() const override
  {
    return *problemPtr_;
  }
  const ModelSettings& modelSettings() const
  {
    return modelSettings_;
  }
  const mpc::Settings& mpcSettings() const
  {
    return mpcSettings_;
  }
  const multiple_shooting::Settings& sqpSettings()
  {
    return sqpSettings_;
  }
  const vector_t& getInitialState() const
  {
    return initialState_;
  }
  const RolloutBase& getRollout() const
  {
    return *rolloutPtr_;
  }
  PinocchioInterface& getPinocchioInterface()
  {
    return *pinocchioInterfacePtr_;
  }
  const CentroidalModelInfo& getCentroidalModelInfo() const
  {
    return centroidalModelInfo_;
  }
  std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const
  {
    return referenceManagerPtr_;
  }
  const LeggedRobotInitializer& getInitializer() const override
  {
    return *initializerPtr_;
  }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override
  {
    return referenceManagerPtr_;
  }

protected:
  virtual void setupModel(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                          bool verbose);

  std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string& file, bool verbose) const;

  std::unique_ptr<StateInputCost> getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info,
                                                      bool verbose);
  matrix_t initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info);

  std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string& taskFile,
                                                                              bool verbose) const;
  std::unique_ptr<StateInputCost> getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                            const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);
  std::unique_ptr<StateInputConstraint> getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                  size_t contactPointIndex);

  ModelSettings modelSettings_;
  mpc::Settings mpcSettings_;
  multiple_shooting::Settings sqpSettings_;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  CentroidalModelInfo centroidalModelInfo_;

  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

  rollout::Settings rolloutSettings_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<LeggedRobotInitializer> initializerPtr_;

  vector_t initialState_;
};

}  // namespace legged
