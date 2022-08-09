//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/jacobian.hpp>

#include "legged_controllers/legged_controller.h"

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_msgs/mpc_observation.h>

#include <legged_estimation/from_topice_estimate.h>
#include <legged_estimation/linear_kalman_filter.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged
{
bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  // Initialize OCS2
  std::string task_file, urdf_file, reference_file;
  controller_nh.getParam("/task_file", task_file);
  controller_nh.getParam("/urdf_file", urdf_file);
  controller_nh.getParam("/reference_file", reference_file);
  bool verbose;
  loadData::loadCppDataType(task_file, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(task_file, urdf_file, reference_file, verbose);
  setupMpc();
  setupMrt();

  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
  PinocchioEndEffectorKinematics ee_kinematics(legged_interface_->getPinocchioInterface(), pinocchio_mapping,
                                               legged_interface_->modelSettings().contactNames3DoF);
  visualizer_ = std::make_shared<LeggedRobotVisualizer>(legged_interface_->getPinocchioInterface(),
                                                        legged_interface_->getCentroidalModelInfo(), ee_kinematics, nh);

  // Hardware interface
  HybridJointInterface* hybrid_joint_interface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{ "LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                        "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE" };
  for (const auto& joint_name : joint_names)
    hybrid_joint_handles_.push_back(hybrid_joint_interface->getHandle(joint_name));

  ContactSensorInterface* contact_interface = robot_hw->get<ContactSensorInterface>();
  std::vector<ContactSensorHandle> contact_handles;
  for (auto& name : legged_interface_->modelSettings().contactNames3DoF)
    contact_handles.push_back(contact_interface->getHandle(name));

  // State estimation
  setupStateEstimate(*legged_interface_, hybrid_joint_handles_, contact_handles,
                     robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu"));

  // Whole body control
  wbc_ = std::make_shared<Wbc>(task_file, *legged_interface_, ee_kinematics, verbose);

  // Safety Checker
  safety_checker_ = std::make_shared<SafetyChecker>(legged_interface_->getCentroidalModelInfo());

  return true;
}

void LeggedController::starting(const ros::Time& time)
{
  // Initial state
  current_observation_.mode = ModeNumber::STANCE;
  current_observation_.state =
      rbd_conversions_->computeCentroidalStateFromRbdModel(state_estimate_->update(time, ros::Duration(0.002)));
  current_observation_.input.setZero(legged_interface_->getCentroidalModelInfo().inputDim);

  TargetTrajectories target_trajectories({ current_observation_.time }, { current_observation_.state },
                                         { current_observation_.input });

  // Set the first observation and command and wait for optimization to finish
  mpc_mrt_interface_->setCurrentObservation(current_observation_);
  mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpc_mrt_interface_->initialPolicyReceived() && ros::ok())
  {
    mpc_mrt_interface_->advanceMpc();
    ros::WallRate(legged_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpc_running_ = true;
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period)
{
  // State Estimate
  current_observation_.time += period.toSec();

  vector_t measured_rbd_state = state_estimate_->update(time, period);
  scalar_t yaw_last = current_observation_.state(9);
  current_observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(measured_rbd_state);
  current_observation_.state(9) = yaw_last + angles::shortest_angular_distance(yaw_last, current_observation_.state(9));
  current_observation_.mode = state_estimate_->getMode();

  // Update the current state of the system
  mpc_mrt_interface_->setCurrentObservation(current_observation_);

  // Load the latest MPC policy
  mpc_mrt_interface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimized_state, optimized_input;
  size_t planned_mode;  // The mode that is active at the time the policy is evaluated at.
  mpc_mrt_interface_->evaluatePolicy(current_observation_.time, current_observation_.state, optimized_state,
                                     optimized_input, planned_mode);

  // Whole body control
  for (size_t i = 0; i < legged_interface_->getCentroidalModelInfo().numThreeDofContacts; i++)
    current_observation_.input.segment<3>(i * 3) =
        centroidal_model::getContactForces(optimized_input, i, legged_interface_->getCentroidalModelInfo());

  vector_t x = wbc_->update(optimized_state, optimized_input, measured_rbd_state, planned_mode);

  vector_t torque = x.tail(12);

  vector_t pos_des = centroidal_model::getJointAngles(optimized_state, legged_interface_->getCentroidalModelInfo());
  vector_t vel_des = centroidal_model::getJointVelocities(optimized_input, legged_interface_->getCentroidalModelInfo());

  // Safety check, if failed, stop the controller
  if (!safety_checker_->check(current_observation_, optimized_state, optimized_input))
  {
    ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
  }

  for (size_t j = 0; j < legged_interface_->getCentroidalModelInfo().actuatedDofNum; ++j)
    hybrid_joint_handles_[j].setCommand(pos_des(j), vel_des(j), 5, 3, torque(j));

  // Visualization
  visualizer_->update(current_observation_, mpc_mrt_interface_->getPolicy(), mpc_mrt_interface_->getCommand());

  // Publish the observation. Only needed for the command interface
  observation_publisher_.publish(ros_msg_conversions::createObservationMsg(current_observation_));
}

LeggedController::~LeggedController()
{
  controller_running_ = false;
  if (mpc_thread_.joinable())
    mpc_thread_.join();
}

void LeggedController::setupLeggedInterface(const std::string& task_file, const std::string& urdf_file,
                                            const std::string& reference_file, bool verbose)
{
  legged_interface_ = std::make_shared<LeggedInterface>(task_file, urdf_file, reference_file, verbose);
  legged_interface_->setupOptimalControlProblem(task_file, urdf_file, reference_file, verbose);
}

void LeggedController::setupMpc()
{
  mpc_ = std::make_shared<MultipleShootingMpc>(legged_interface_->mpcSettings(), legged_interface_->sqpSettings(),
                                               legged_interface_->getOptimalControlProblem(),
                                               legged_interface_->getInitializer());
  rbd_conversions_ = std::make_shared<CentroidalModelRbdConversions>(legged_interface_->getPinocchioInterface(),
                                                                     legged_interface_->getCentroidalModelInfo());

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
  observation_publisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robot_name + "_mpc_observation", 1);
}

void LeggedController::setupMrt()
{
  mpc_mrt_interface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());

  controller_running_ = true;
  mpc_thread_ = std::thread([&]() {
    while (controller_running_)
    {
      try
      {
        executeAndSleep(
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
  setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpc_thread_);
}

void LeggedController::setupStateEstimate(LeggedInterface& legged_interface,
                                          const std::vector<HybridJointHandle>& hybrid_joint_handles,
                                          const std::vector<ContactSensorHandle>& contact_sensor_handles,
                                          const hardware_interface::ImuSensorHandle& imu_sensor_handle)
{
  state_estimate_ = std::make_shared<KalmanFilterEstimate>(*legged_interface_, hybrid_joint_handles_,
                                                           contact_sensor_handles, imu_sensor_handle);
  current_observation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(LeggedInterface& legged_interface,
                                                 const std::vector<HybridJointHandle>& hybrid_joint_handles,
                                                 const std::vector<ContactSensorHandle>& contact_sensor_handles,
                                                 const hardware_interface::ImuSensorHandle& imu_sensor_handle)
{
  state_estimate_ = std::make_shared<FromTopicStateEstimate>(*legged_interface_, hybrid_joint_handles_,
                                                             contact_sensor_handles, imu_sensor_handle);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
