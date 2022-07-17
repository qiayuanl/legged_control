//
// Created by qiayuan on 2022/7/16.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "legged_interface/legged_interface.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <ocs2_legged_robot/LeggedRobotPreComputation.h>
#include <ocs2_legged_robot/constraint/FrictionConeConstraint.h>
#include <ocs2_legged_robot/constraint/NormalVelocityConstraintCppAd.h>
#include <ocs2_legged_robot/constraint/ZeroForceConstraint.h>
#include <ocs2_legged_robot/constraint/ZeroVelocityConstraintCppAd.h>
#include <ocs2_legged_robot/cost/LeggedRobotStateInputQuadraticCost.h>
#include <ocs2_legged_robot/dynamics/LeggedRobotDynamicsAD.h>

namespace legged
{
LeggedInterface::LeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                 const std::string& referenceFile)
{
  // check that task file exists
  boost::filesystem::path task_file_path(taskFile);
  if (boost::filesystem::exists(task_file_path))
    std::cerr << "[LeggedInterface] Loading task file: " << task_file_path << std::endl;
  else
    throw std::invalid_argument("[LeggedInterface] Task file not found: " + task_file_path.string());

  // check that urdf file exists
  boost::filesystem::path urdf_file_path(urdfFile);
  if (boost::filesystem::exists(urdf_file_path))
    std::cerr << "[LeggedInterface] Loading Pinocchio model from: " << urdf_file_path << std::endl;
  else
    throw std::invalid_argument("[LeggedInterface] URDF file not found: " + urdf_file_path.string());

  // check that targetCommand file exists
  boost::filesystem::path reference_file_path(referenceFile);
  if (boost::filesystem::exists(reference_file_path))
    std::cerr << "[LeggedInterface] Loading target command settings from: " << reference_file_path << std::endl;
  else
    throw std::invalid_argument("[LeggedInterface] targetCommand file not found: " + reference_file_path.string());

  bool verbose;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  // load setting from loading file
  modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
  rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);
  sqpSettings_ = multiple_shooting::loadSettings(taskFile, "multiple_shooting", verbose);

  // OptimalControlProblem
  setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  // initial state
  initialState_.setZero(centroidalModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                 const std::string& referenceFile, bool verbose)
{
  // PinocchioInterface
  pinocchioInterfacePtr_.reset(
      new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames)));

  // CentroidalModelInfo
  centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
      *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
      centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile),
      modelSettings_.contactNames3DoF, modelSettings_.contactNames6DoF);

  // Swing trajectory planner
  std::unique_ptr<SwingTrajectoryPlanner> swing_trajectory_planner(
      new SwingTrajectoryPlanner(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4));

  // Mode schedule manager
  referenceManagerPtr_ = std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(referenceFile, verbose),
                                                                         std::move(swing_trajectory_planner));

  // Optimal control problem
  problemPtr_.reset(new OptimalControlProblem);

  // Dynamics
  std::unique_ptr<SystemDynamicsBase> dynamics_ptr;
  dynamics_ptr.reset(
      new LeggedRobotDynamicsAD(*pinocchioInterfacePtr_, centroidalModelInfo_, "dynamics", modelSettings_));
  problemPtr_->dynamicsPtr = std::move(dynamics_ptr);

  // Cost terms
  problemPtr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(taskFile, centroidalModelInfo_, verbose));

  // Constraint terms
  // friction cone settings
  scalar_t friction_coefficient = 0.7;
  RelaxedBarrierPenalty::Config barrier_penalty_config;
  std::tie(friction_coefficient, barrier_penalty_config) = loadFrictionConeSettings(taskFile, verbose);

  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
  {
    const std::string& foot_name = modelSettings_.contactNames3DoF[i];

    std::unique_ptr<EndEffectorKinematics<scalar_t>> ee_kinematics_ptr;

    const auto info_cpp_ad = centroidalModelInfo_.toCppAd();
    const CentroidalModelPinocchioMappingCppAd pinocchio_mapping_cpp_ad(info_cpp_ad);
    auto velocity_update_callback = [&info_cpp_ad](const ad_vector_t& state,
                                                   PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
      const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, info_cpp_ad);
      updateCentroidalDynamics(pinocchioInterfaceAd, info_cpp_ad, q);
    };
    ee_kinematics_ptr.reset(new PinocchioEndEffectorKinematicsCppAd(
        *pinocchioInterfacePtr_, pinocchio_mapping_cpp_ad, { foot_name }, centroidalModelInfo_.stateDim,
        centroidalModelInfo_.inputDim, velocity_update_callback, foot_name, modelSettings_.modelFolderCppAd,
        modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));

    problemPtr_->softConstraintPtr->add(foot_name + "_frictionCone",
                                        getFrictionConeConstraint(i, friction_coefficient, barrier_penalty_config));
    problemPtr_->equalityConstraintPtr->add(
        foot_name + "_zeroForce",
        std::unique_ptr<StateInputConstraint>(new ZeroForceConstraint(*referenceManagerPtr_, i, centroidalModelInfo_)));
    problemPtr_->equalityConstraintPtr->add(foot_name + "_zeroVelocity",
                                            getZeroVelocityConstraint(*ee_kinematics_ptr, i));
    problemPtr_->equalityConstraintPtr->add(foot_name + "_normalVelocity",
                                            std::unique_ptr<StateInputConstraint>(new NormalVelocityConstraintCppAd(
                                                *referenceManagerPtr_, *ee_kinematics_ptr, i)));
  }

  // Pre-computation
  problemPtr_->preComputationPtr.reset(new LeggedRobotPreComputation(*pinocchioInterfacePtr_, centroidalModelInfo_,
                                                                     *referenceManagerPtr_->getSwingTrajectoryPlanner(),
                                                                     modelSettings_));

  // Rollout
  rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

  // Initialization
  constexpr bool extend_normalized_momentum = true;
  initializerPtr_.reset(
      new LeggedRobotInitializer(centroidalModelInfo_, *referenceManagerPtr_, extend_normalized_momentum));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<GaitSchedule> LeggedInterface::loadGaitSchedule(const std::string& file, bool verbose) const
{
  const auto init_mode_schedule = loadModeSchedule(file, "initialModeSchedule", false);
  const auto default_mode_sequence_template = loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

  const auto default_gait = [default_mode_sequence_template] {
    Gait gait{};
    gait.duration = default_mode_sequence_template.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(default_mode_sequence_template.switchingTimes.begin() + 1,
                  default_mode_sequence_template.switchingTimes.end() - 1,
                  [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = default_mode_sequence_template.modeSequence;
    return gait;
  }();

  // display
  if (verbose)
  {
    std::cerr << "\n#### Modes Schedule: ";
    std::cerr << "\n#### =============================================================================\n";
    std::cerr << "Initial Modes Schedule: \n" << init_mode_schedule;
    std::cerr << "Default Modes Sequence Template: \n" << default_mode_sequence_template;
    std::cerr << "#### =============================================================================\n";
  }

  return std::make_shared<GaitSchedule>(init_mode_schedule, default_mode_sequence_template,
                                        modelSettings_.phaseTransitionStanceTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LeggedInterface::initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info)
{
  const size_t total_contact_dim = 3 * info.numThreeDofContacts;

  vector_t initial_state(centroidalModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initial_state);

  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto q = centroidal_model::getGeneralizedCoordinates(initial_state, centroidalModelInfo_);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  matrix_t base2feet_jac(total_contact_dim, info.actuatedDofNum);
  for (size_t i = 0; i < info.numThreeDofContacts; i++)
  {
    matrix_t jac = matrix_t::Zero(6, info.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, model.getBodyId(modelSettings_.contactNames3DoF[i]),
                                pinocchio::LOCAL_WORLD_ALIGNED, jac);
    base2feet_jac.block(3 * i, 0, 3, info.actuatedDofNum) = jac.block(0, 6, 3, info.actuatedDofNum);
  }

  matrix_t r_taskspace(total_contact_dim + total_contact_dim, total_contact_dim + total_contact_dim);
  loadData::loadEigenMatrix(taskFile, "R", r_taskspace);

  matrix_t r = matrix_t::Zero(info.inputDim, info.inputDim);
  // Contact Forces
  r.topLeftCorner(total_contact_dim, total_contact_dim) =
      r_taskspace.topLeftCorner(total_contact_dim, total_contact_dim);
  // Joint velocities
  r.bottomRightCorner(info.actuatedDofNum, info.actuatedDofNum) =
      base2feet_jac.transpose() * r_taskspace.bottomRightCorner(total_contact_dim, total_contact_dim) * base2feet_jac;
  return r;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedInterface::getBaseTrackingCost(const std::string& taskFile,
                                                                     const CentroidalModelInfo& info, bool verbose)
{
  matrix_t q(info.stateDim, info.stateDim);
  loadData::loadEigenMatrix(taskFile, "Q", q);
  matrix_t r = initializeInputCostWeight(taskFile, info);

  if (verbose)
  {
    std::cerr << "\n #### Base Tracking Cost Coefficients: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "Q:\n" << q << "\n";
    std::cerr << "R:\n" << r << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  return std::unique_ptr<StateInputCost>(
      new LeggedRobotStateInputQuadraticCost(std::move(q), std::move(r), info, *referenceManagerPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<scalar_t, RelaxedBarrierPenalty::Config>
LeggedInterface::loadFrictionConeSettings(const std::string& taskFile, bool verbose) const
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "frictionConeSoftConstraint.";

  scalar_t friction_coefficient = 1.0;
  RelaxedBarrierPenalty::Config barrier_penalty_config;
  if (verbose)
  {
    std::cerr << "\n #### Friction Cone Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, friction_coefficient, prefix + "frictionCoefficient", verbose);
  loadData::loadPtreeValue(pt, barrier_penalty_config.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrier_penalty_config.delta, prefix + "delta", verbose);
  if (verbose)
  {
    std::cerr << " #### =============================================================================\n";
  }

  return { friction_coefficient, barrier_penalty_config };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost>
LeggedInterface::getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                           const RelaxedBarrierPenalty::Config& barrierPenaltyConfig)
{
  FrictionConeConstraint::Config friction_cone_con_config(frictionCoefficient);
  std::unique_ptr<FrictionConeConstraint> friction_cone_constraint_ptr(new FrictionConeConstraint(
      *referenceManagerPtr_, friction_cone_con_config, contactPointIndex, centroidalModelInfo_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(
      new StateInputSoftConstraint(std::move(friction_cone_constraint_ptr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint>
LeggedInterface::getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics, size_t contactPointIndex)
{
  auto ee_zero_vel_con_config = [](scalar_t positionErrorGain) {
    EndEffectorLinearConstraint::Config config;
    config.b.setZero(3);
    config.Av.setIdentity(3, 3);
    if (!numerics::almost_eq(positionErrorGain, 0.0))
    {
      config.Ax.setZero(3, 3);
      config.Ax(2, 2) = positionErrorGain;
    }
    return config;
  };
  return std::unique_ptr<StateInputConstraint>(
      new ZeroVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex,
                                      ee_zero_vel_con_config(modelSettings_.positionErrorGain)));
}

}  // namespace legged
