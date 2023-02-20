//
// Created by qiayuan on 2022/7/16.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "legged_interface/LeggedInterface.h"
#include "legged_interface/LeggedRobotPreComputation.h"
#include "legged_interface/constraint/FrictionConeConstraint.h"
#include "legged_interface/constraint/LeggedSelfCollisionConstraint.h"
#include "legged_interface/constraint/NormalVelocityConstraintCppAd.h"
#include "legged_interface/constraint/ZeroForceConstraint.h"
#include "legged_interface/constraint/ZeroVelocityConstraintCppAd.h"
#include "legged_interface/cost/LeggedRobotQuadraticTrackingCost.h"
#include "legged_interface/initialization/LeggedRobotInitializer.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <ocs2_legged_robot/dynamics/LeggedRobotDynamicsAD.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace legged {
LeggedInterface::LeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                 bool useHardFrictionConeConstraint)
    : useHardFrictionConeConstraint_(useHardFrictionConeConstraint) {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[LeggedInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedInterface] Task file not found: " + taskFilePath.string());
  }

  // check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[LeggedInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedInterface] URDF file not found: " + urdfFilePath.string());
  }

  // check that targetCommand file exists
  boost::filesystem::path referenceFilePath(referenceFile);
  if (boost::filesystem::exists(referenceFilePath)) {
    std::cerr << "[LeggedInterface] Loading target command settings from: " << referenceFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedInterface] targetCommand file not found: " + referenceFilePath.string());
  }

  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  // load setting from loading file
  modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
  sqpSettings_ = sqp::loadSettings(taskFile, "sqp", verbose);
  ipmSettings_ = ipm::loadSettings(taskFile, "ipm", verbose);
  rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                                 bool verbose) {
  setupModel(taskFile, urdfFile, referenceFile, verbose);

  // Initial state
  initialState_.setZero(centroidalModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

  setupReferenceManager(taskFile, urdfFile, referenceFile, verbose);

  // Optimal control problem
  problemPtr_ = std::make_unique<OptimalControlProblem>();

  // Dynamics
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  dynamicsPtr = std::make_unique<LeggedRobotDynamicsAD>(*pinocchioInterfacePtr_, centroidalModelInfo_, "dynamics", modelSettings_);
  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

  // Cost terms
  problemPtr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(taskFile, centroidalModelInfo_, verbose));

  // Constraint terms
  // friction cone settings
  scalar_t frictionCoefficient = 0.7;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  std::tie(frictionCoefficient, barrierPenaltyConfig) = loadFrictionConeSettings(taskFile, verbose);

  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    const std::string& footName = modelSettings_.contactNames3DoF[i];
    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({footName}, footName);

    if (useHardFrictionConeConstraint_) {
      problemPtr_->inequalityConstraintPtr->add(footName + "_frictionCone", getFrictionConeConstraint(i, frictionCoefficient));
    } else {
      problemPtr_->softConstraintPtr->add(footName + "_frictionCone",
                                          getFrictionConeSoftConstraint(i, frictionCoefficient, barrierPenaltyConfig));
    }
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroForce", std::unique_ptr<StateInputConstraint>(new ZeroForceConstraint(
                                                                         *referenceManagerPtr_, i, centroidalModelInfo_)));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity", getZeroVelocityConstraint(*eeKinematicsPtr, i));
    problemPtr_->equalityConstraintPtr->add(
        footName + "_normalVelocity",
        std::unique_ptr<StateInputConstraint>(new NormalVelocityConstraintCppAd(*referenceManagerPtr_, *eeKinematicsPtr, i)));
  }

  // Self-collision avoidance constraint
  problemPtr_->stateSoftConstraintPtr->add("selfCollision",
                                           getSelfCollisionConstraint(*pinocchioInterfacePtr_, taskFile, "selfCollision", verbose));

  setupPreComputation(taskFile, urdfFile, referenceFile, verbose);

  // Rollout
  rolloutPtr_ = std::make_unique<TimeTriggeredRollout>(*problemPtr_->dynamicsPtr, rolloutSettings_);

  // Initialization
  constexpr bool extendNormalizedNomentum = true;
  initializerPtr_ = std::make_unique<LeggedRobotInitializer>(centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedNomentum);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedInterface::setupModel(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                 bool /*verbose*/) {
  // PinocchioInterface
  pinocchioInterfacePtr_ =
      std::make_unique<PinocchioInterface>(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames));

  // CentroidalModelInfo
  centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
      *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
      centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile), modelSettings_.contactNames3DoF,
      modelSettings_.contactNames6DoF);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedInterface::setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  auto swingTrajectoryPlanner =
      std::make_unique<SwingTrajectoryPlanner>(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);
  referenceManagerPtr_ =
      std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(referenceFile, verbose), std::move(swingTrajectoryPlanner));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedInterface::setupPreComputation(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                          bool verbose) {
  problemPtr_->preComputationPtr = std::make_unique<LeggedRobotPreComputation>(
      *pinocchioInterfacePtr_, centroidalModelInfo_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<GaitSchedule> LeggedInterface::loadGaitSchedule(const std::string& file, bool verbose) const {
  const auto initModeSchedule = loadModeSchedule(file, "initialModeSchedule", false);
  const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

  const auto defaultGait = [defaultModeSequenceTemplate] {
    Gait gait{};
    gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1, defaultModeSequenceTemplate.switchingTimes.end() - 1,
                  [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
    return gait;
  }();

  // display
  if (verbose) {
    std::cerr << "\n#### Modes Schedule: ";
    std::cerr << "\n#### =============================================================================\n";
    std::cerr << "Initial Modes Schedule: \n" << initModeSchedule;
    std::cerr << "Default Modes Sequence Template: \n" << defaultModeSequenceTemplate;
    std::cerr << "#### =============================================================================\n";
  }

  return std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, modelSettings_.phaseTransitionStanceTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LeggedInterface::initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info) {
  const size_t totalContactDim = 3 * info.numThreeDofContacts;

  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto q = centroidal_model::getGeneralizedCoordinates(initialState_, centroidalModelInfo_);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  matrix_t base2feetJac(totalContactDim, info.actuatedDofNum);
  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    matrix_t jac = matrix_t::Zero(6, info.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, model.getBodyId(modelSettings_.contactNames3DoF[i]), pinocchio::LOCAL_WORLD_ALIGNED, jac);
    base2feetJac.block(3 * i, 0, 3, info.actuatedDofNum) = jac.block(0, 6, 3, info.actuatedDofNum);
  }

  matrix_t rTaskspace(info.inputDim, info.inputDim);
  loadData::loadEigenMatrix(taskFile, "R", rTaskspace);
  matrix_t r = rTaskspace;
  // Joint velocities
  r.block(totalContactDim, totalContactDim, info.actuatedDofNum, info.actuatedDofNum) =
      base2feetJac.transpose() * rTaskspace.block(totalContactDim, totalContactDim, info.actuatedDofNum, info.actuatedDofNum) *
      base2feetJac;
  return r;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedInterface::getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info,
                                                                     bool verbose) {
  matrix_t Q(info.stateDim, info.stateDim);
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  matrix_t R = initializeInputCostWeight(taskFile, info);

  if (verbose) {
    std::cerr << "\n #### Base Tracking Cost Coefficients: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "Q:\n" << Q << "\n";
    std::cerr << "R:\n" << R << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  return std::make_unique<LeggedRobotStateInputQuadraticCost>(std::move(Q), std::move(R), info, *referenceManagerPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<scalar_t, RelaxedBarrierPenalty::Config> LeggedInterface::loadFrictionConeSettings(const std::string& taskFile, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "frictionConeSoftConstraint.";

  scalar_t frictionCoefficient = 1.0;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Friction Cone Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return {frictionCoefficient, barrierPenaltyConfig};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedInterface::getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient) {
  FrictionConeConstraint::Config frictionConeConConfig(frictionCoefficient);
  return std::make_unique<FrictionConeConstraint>(*referenceManagerPtr_, frictionConeConConfig, contactPointIndex, centroidalModelInfo_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedInterface::getFrictionConeSoftConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                                               const RelaxedBarrierPenalty::Config& barrierPenaltyConfig) {
  return std::make_unique<StateInputSoftConstraint>(getFrictionConeConstraint(contactPointIndex, frictionCoefficient),
                                                    std::make_unique<RelaxedBarrierPenalty>(barrierPenaltyConfig));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<EndEffectorKinematics<scalar_t>> LeggedInterface::getEeKinematicsPtr(const std::vector<std::string>& footNames,
                                                                                     const std::string& modelName) {
  std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;

  const auto infoCppAd = centroidalModelInfo_.toCppAd();
  const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
  auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
    const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
    updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
  };
  eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, footNames,
                                                                centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                velocityUpdateCallback, modelName, modelSettings_.modelFolderCppAd,
                                                                modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));

  return eeKinematicsPtr;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedInterface::getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                 size_t contactPointIndex) {
  auto eeZeroVelConConfig = [](scalar_t positionErrorGain) {
    EndEffectorLinearConstraint::Config config;
    config.b.setZero(3);
    config.Av.setIdentity(3, 3);
    if (!numerics::almost_eq(positionErrorGain, 0.0)) {
      config.Ax.setZero(3, 3);
      config.Ax(2, 2) = positionErrorGain;
    }
    return config;
  };
  return std::unique_ptr<StateInputConstraint>(new ZeroVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex,
                                                                               eeZeroVelConConfig(modelSettings_.positionErrorGain)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> LeggedInterface::getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface,
                                                                       const std::string& taskFile, const std::string& prefix,
                                                                       bool verbose) {
  std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t minimumDistance = 0.0;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  if (verbose) {
    std::cerr << "\n #### SelfCollision Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, mu, prefix + ".mu", verbose);
  loadData::loadPtreeValue(pt, delta, prefix + ".delta", verbose);
  loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", verbose);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, verbose);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, verbose);

  geometryInterfacePtr_ = std::make_unique<PinocchioGeometryInterface>(pinocchioInterface, collisionLinkPairs, collisionObjectPairs);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
    const size_t numCollisionPairs = geometryInterfacePtr_->getNumCollisionPairs();
    std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";
  }

  std::unique_ptr<StateConstraint> constraint = std::make_unique<LeggedSelfCollisionConstraint>(
      CentroidalModelPinocchioMapping(centroidalModelInfo_), *geometryInterfacePtr_, minimumDistance);

  auto penalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{mu, delta});

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
}

}  // namespace legged
