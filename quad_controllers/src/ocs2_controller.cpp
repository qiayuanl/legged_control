//
// Created by qiayuan on 2022/6/24.
//
#include "quad_controllers/ocs2_controller.h"

#include <pluginlib/class_list_macros.hpp>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>

namespace quad_ros
{
bool Ocs2Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  std::string task_file, urdf_file, reference_file;
  controller_nh.getParam("/task_file", task_file);
  controller_nh.getParam("/urdf_file", urdf_file);
  controller_nh.getParam("/reference_file", reference_file);

  // Robot interface
  legged_interface_ = std::make_shared<LeggedRobotInterface>(task_file, urdf_file, reference_file);
  mpc_ = std::make_shared<MultipleShootingMpc>(legged_interface_->mpcSettings(), legged_interface_->sqpSettings(),
                                               legged_interface_->getOptimalControlProblem(),
                                               legged_interface_->getInitializer());
  const std::string robot_name = "legged_robot";
  ros::NodeHandle nh;
  // Gait receiver
  auto gait_receiver_ptr = std::make_shared<GaitReceiver>(
      nh, legged_interface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robot_name);
  // ROS ReferenceManager
  auto ros_reference_manager_ptr =
      std::make_shared<RosReferenceManager>(robot_name, legged_interface_->getReferenceManagerPtr());
  ros_reference_manager_ptr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gait_receiver_ptr);
  mpc_->getSolverPtr()->setReferenceManager(ros_reference_manager_ptr);

  // State Estimate
  state_estimate_ = std::make_shared<FromTopicStateEstimate>(nh, legged_interface_->getPinocchioInterface(),
                                                             legged_interface_->getCentroidalModelInfo());

  // Visualization
  CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
  PinocchioEndEffectorKinematics ee_kinematics(legged_interface_->getPinocchioInterface(), pinocchio_mapping,
                                               legged_interface_->modelSettings().contactNames3DoF);
  visualizer_ = std::make_shared<LeggedRobotVisualizer>(legged_interface_->getPinocchioInterface(),
                                                        legged_interface_->getCentroidalModelInfo(), ee_kinematics, nh);

  // Create the MPC MRT Interface
  mpc_mrt_interface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());

  // Initial state
  SystemObservation init_observation;
  init_observation.state = legged_interface_->getInitialState();
  init_observation.input = vector_t::Zero(legged_interface_->getCentroidalModelInfo().inputDim);
  init_observation.mode = ModeNumber::STANCE;
  current_observation_ = init_observation;

  // Initial command
  TargetTrajectories init_target_trajectories({ 0.0 }, { init_observation.state }, { init_observation.input });

  // Set the first observation and command and wait for optimization to finish
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  mpc_mrt_interface_->setCurrentObservation(init_observation);
  mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(init_target_trajectories);
  while (!mpc_mrt_interface_->initialPolicyReceived() && ros::ok() && ros::master::check())
  {
    mpc_mrt_interface_->advanceMpc();
    ros::WallRate(legged_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  controller_running_ = true;
  mpc_thread_ = std::thread([&]() {
    while (controller_running_)
    {
      try
      {
        ocs2::executeAndSleep(
            [&]() {
              if (mpc_running_)
                mpc_mrt_interface_->advanceMpc();
            },
            legged_interface_->mpcSettings().mpcDesiredFrequency_);
      }
      catch (const std::exception& e)
      {
        controller_running_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
      }
    }
  });
  ocs2::setThreadPriority(legged_interface_->ddpSettings().threadPriority_, mpc_thread_);

  return true;
}

void Ocs2Controller::update(const ros::Time& time, const ros::Duration& period)
{
  // State Estimate
  current_observation_.time = current_observation_.time + period.toSec();
  current_observation_.state = state_estimate_->update(time).state;

  // Update the current state of the system
  mpc_mrt_interface_->setCurrentObservation(current_observation_);

  // Load the latest MPC policy
  mpc_mrt_interface_->updatePolicy();

  // Evaluate the current policy
  ocs2::vector_t optimized_state;  // Evaluation of the optimized state trajectory.
  ocs2::vector_t optimized_input;  // Evaluation of the optimized input trajectory.
  size_t planned_mode;             // The mode that is active at the time the policy is
                                   // evaluated at.
  mpc_mrt_interface_->evaluatePolicy(current_observation_.time, current_observation_.state, optimized_state,
                                     optimized_input, planned_mode);

  // Visualization
  visualizer_->update(current_observation_, mpc_mrt_interface_->getPolicy(), mpc_mrt_interface_->getCommand());
}

Ocs2Controller::~Ocs2Controller()
{
  controller_running_ = false;
  if (mpc_thread_.joinable())
    mpc_thread_.join();
}

}  // namespace quad_ros

PLUGINLIB_EXPORT_CLASS(quad_ros::Ocs2Controller, controller_interface::ControllerBase)
