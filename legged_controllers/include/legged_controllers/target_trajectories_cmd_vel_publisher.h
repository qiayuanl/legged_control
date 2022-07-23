//
// Created by qiayuan on 2022/7/23.
//

#pragma once

#include <mutex>
#include <ros/subscriber.h>
#include <geometry_msgs/Twist.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace legged
{
using namespace ocs2;

class TargetTrajectoriesGoalPublisher final
{
public:
  using CmdVelToTargetTrajectories =
      std::function<TargetTrajectories(const vector_t& cmd_vel, const SystemObservation& observation)>;

  TargetTrajectoriesGoalPublisher(::ros::NodeHandle& nh, const std::string& topic_prefix,
                                  CmdVelToTargetTrajectories goal_to_target_trajectories)
    : cmd_vel_to_target_trajectories_(std::move(goal_to_target_trajectories))
  {
    // Trajectories publisher
    target_trajectories_publisher_.reset(new TargetTrajectoriesRosPublisher(nh, topic_prefix));

    // observation subscriber
    auto observation_callback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(latest_observation_mutex_);
      latest_observation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    observation_sub_ =
        nh.subscribe<ocs2_msgs::mpc_observation>(topic_prefix + "_mpc_observation", 1, observation_callback);

    // cmd_vel subscriber
    auto cmd_vel_callback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
      vector_t cmd_vel = vector_t::Zero(6);
      cmd_vel[0] = msg->linear.x;
      cmd_vel[1] = msg->linear.y;
      cmd_vel[2] = msg->linear.z;
      cmd_vel[3] = msg->angular.x;
      cmd_vel[4] = msg->angular.y;
      cmd_vel[5] = msg->angular.z;

      const auto trajectories = cmd_vel_to_target_trajectories_(cmd_vel, latest_observation_);
      target_trajectories_publisher_->publishTargetTrajectories(trajectories);
    };
    cmd_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmd_vel_callback);
  }

private:
  CmdVelToTargetTrajectories cmd_vel_to_target_trajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> target_trajectories_publisher_;

  ::ros::Subscriber observation_sub_, cmd_vel_sub_;
  mutable std::mutex latest_observation_mutex_;
  SystemObservation latest_observation_;
};

}  // namespace legged
