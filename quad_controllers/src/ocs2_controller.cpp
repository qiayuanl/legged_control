//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/jacobian.hpp>

#include "quad_controllers/ocs2_controller.h"

#include <pluginlib/class_list_macros.hpp>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_msgs/mpc_observation.h>
#include <angles/angles.h>

namespace quad_ros
{
bool Ocs2Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  // Initialize OCS2
  std::string task_file, urdf_file, reference_file;
  controller_nh.getParam("/task_file", task_file);
  controller_nh.getParam("/urdf_file", urdf_file);
  controller_nh.getParam("/referenceFile", reference_file);

  // Robot interface
  legged_interface_ = std::make_shared<LeggedRobotInterface>(task_file, urdf_file, reference_file);
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

  // Visualization
  CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
  PinocchioEndEffectorKinematics ee_kinematics(legged_interface_->getPinocchioInterface(), pinocchio_mapping,
                                               legged_interface_->modelSettings().contactNames3DoF);
  visualizer_ = std::make_shared<LeggedRobotVisualizer>(legged_interface_->getPinocchioInterface(),
                                                        legged_interface_->getCentroidalModelInfo(), ee_kinematics, nh);

  // Create the MPC MRT Interface
  mpc_mrt_interface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());

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

  HybridJointInterface* hybrid_joint_interface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{ "LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                        "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE" };
  for (const auto& joint_name : joint_names)
    hybrid_joint_handles_.push_back(hybrid_joint_interface->getHandle(joint_name));

  ContactSensorInterface* contact_interface = robot_hw->get<ContactSensorInterface>();
  std::vector<ContactSensorHandle> contact_handles;
  for (auto& name : legged_interface_->modelSettings().contactNames3DoF)
    contact_handles.push_back(contact_interface->getHandle(name));
  state_estimate_ = std::make_shared<KalmanFilterEstimate>(
      nh, *legged_interface_, hybrid_joint_handles_, contact_handles,
      robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu"));

  wbc_ = std::make_shared<Wbc>(*legged_interface_);
  return true;
}

void Ocs2Controller::starting(const ros::Time& time)
{
  // Initial state
  current_observation_.mode = ModeNumber::STANCE;
  current_observation_.time = 0;
  current_observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(state_estimate_->update(0.001));
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

void Ocs2Controller::update(const ros::Time& time, const ros::Duration& period)
{
  // State Estimate
  current_observation_.time += period.toSec();

  vector_t measured_rbd_state = state_estimate_->update(period.toSec());
  scalar_t yaw_last = current_observation_.state(9);
  current_observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(measured_rbd_state);
  current_observation_.state(9) = yaw_last + angles::shortest_angular_distance(yaw_last, current_observation_.state(9));
  current_observation_.mode = state_estimate_->getMode();

  // Update the current state of the system
  mpc_mrt_interface_->setCurrentObservation(current_observation_);

  // Load the latest MPC policy
  mpc_mrt_interface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimized_state;  // Evaluation of the optimized state trajectory.
  vector_t optimized_input;  // Evaluation of the optimized input trajectory.
  size_t planned_mode;       // The mode that is active at the time the policy is evaluated at.
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

  for (size_t j = 0; j < legged_interface_->getCentroidalModelInfo().actuatedDofNum; ++j)
    hybrid_joint_handles_[j].setCommand(pos_des(j), vel_des(j), 5, 1, torque(j));

  // Visualization
  visualizer_->update(current_observation_, mpc_mrt_interface_->getPolicy(), mpc_mrt_interface_->getCommand());

  // Publish the observation. Only needed for the command interface
  observation_publisher_.publish(ocs2::ros_msg_conversions::createObservationMsg(current_observation_));
}

Ocs2Controller::~Ocs2Controller()
{
  controller_running_ = false;
  if (mpc_thread_.joinable())
    mpc_thread_.join();
}

}  // namespace quad_ros

PLUGINLIB_EXPORT_CLASS(quad_ros::Ocs2Controller, controller_interface::ControllerBase)
