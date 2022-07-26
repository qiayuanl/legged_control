//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include <mutex>
#include <ros/subscriber.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace legged
{
using namespace ocs2;

class TargetTrajectoriesPublisher final
{
public:
  using CmdToTargetTrajectories =
      std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;

  TargetTrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& topic_prefix,
                              CmdToTargetTrajectories goal_to_target_trajectories,
                              CmdToTargetTrajectories cmd_vel_to_target_trajectories)
    : goal_to_target_trajectories_(std::move(goal_to_target_trajectories))
    , cmd_vel_to_target_trajectories_(std::move(cmd_vel_to_target_trajectories))
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

    // goal subscriber
    auto goal_callback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
      if (latest_observation_.time == 0.0)
        return;

      vector_t cmd_goal = vector_t::Zero(6);
      cmd_goal[0] = msg->pose.position.x;
      cmd_goal[1] = msg->pose.position.y;
      cmd_goal[2] = msg->pose.position.z;
      Eigen::Quaternion<scalar_t> q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                                    msg->pose.orientation.z);
      cmd_goal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
      cmd_goal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
      cmd_goal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

      const auto trajectories = goal_to_target_trajectories_(cmd_goal, latest_observation_);
      target_trajectories_publisher_->publishTargetTrajectories(trajectories);
    };

    // cmd_vel subscriber
    auto cmd_vel_callback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
      if (latest_observation_.time == 0.0)
        return;

      vector_t cmd_vel = vector_t::Zero(4);
      cmd_vel[0] = msg->linear.x;
      cmd_vel[1] = msg->linear.y;
      cmd_vel[2] = msg->linear.z;
      cmd_vel[3] = msg->angular.z;

      const auto trajectories = cmd_vel_to_target_trajectories_(cmd_vel, latest_observation_);
      target_trajectories_publisher_->publishTargetTrajectories(trajectories);
    };

    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goal_callback);
    cmd_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmd_vel_callback);
  }

private:
  CmdToTargetTrajectories goal_to_target_trajectories_, cmd_vel_to_target_trajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> target_trajectories_publisher_;

  ::ros::Subscriber observation_sub_, goal_sub_, cmd_vel_sub_;
  mutable std::mutex latest_observation_mutex_;
  SystemObservation latest_observation_;
};

}  // namespace legged
