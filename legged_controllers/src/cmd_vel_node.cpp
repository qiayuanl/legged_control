//
// Created by qiayuan on 2022/7/18.
//
#include "legged_controllers/target_trajectories_cmd_vel_publisher.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>

using namespace legged;

namespace
{
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
vector_t DEFAULT_JOINT_STATE(12);
scalar_t TIME_TO_TARGET;
}  // namespace

TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmd_vel, const SystemObservation& observation)
{
  // TODO set time_to_target as a parameters
  const scalar_t time_to_target = 1.0;
  const vector_t current_pose = observation.state.segment<6>(6);

  const vector_t target_pose = [&]() {
    vector_t target(6);
    target(0) = current_pose(0) + cmd_vel(0) * time_to_target;
    target(1) = current_pose(1) + cmd_vel(1) * time_to_target;
    target(2) = COM_HEIGHT;
    target(3) = current_pose(3) + cmd_vel(5) * time_to_target;
    target(4) = 0;
    target(5) = 0;
    return target;
  }();

  // target reaching duration

  const scalar_t target_reaching_time = observation.time + time_to_target;

  // desired time trajectory
  const scalar_array_t time_trajectory{ observation.time, target_reaching_time };

  // desired state trajectory
  vector_array_t state_trajectory(2, vector_t::Zero(observation.state.size()));
  state_trajectory[0] << vector_t::Zero(6), current_pose, DEFAULT_JOINT_STATE;
  state_trajectory[1] << vector_t::Zero(6), target_pose, DEFAULT_JOINT_STATE;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t input_trajectory(2, vector_t::Zero(observation.input.size()));

  return { time_trajectory, state_trajectory, input_trajectory };
}

int main(int argc, char* argv[])
{
  const std::string robot_name = "legged_robot";

  // Initialize ros node
  ::ros::init(argc, argv, robot_name + "_target");
  ::ros::NodeHandle node_handle;
  // Get node parameters
  std::string reference_file, task_file;
  node_handle.getParam("/reference_file", reference_file);
  node_handle.getParam("/task_file", task_file);

  loadData::loadCppDataType(reference_file, "comHeight", COM_HEIGHT);
  loadData::loadEigenMatrix(reference_file, "defaultJointState", DEFAULT_JOINT_STATE);
  loadData::loadCppDataType(reference_file, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(reference_file, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(task_file, "mpc.timeHorizon", TIME_TO_TARGET);

  TargetTrajectoriesGoalPublisher target_pose_command(node_handle, robot_name, &cmdVelToTargetTrajectories);

  ros::spin();
  // Successful exit
  return 0;
}
