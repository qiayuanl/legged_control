//
// Created by qiayuan on 2022/7/18.
//

#pragma once

#include <mutex>
#include <ros/subscriber.h>
#include <geometry_msgs/PoseStamped.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace legged
{
using namespace ocs2;

class TargetTrajectoriesGoalPublisher final
{
public:
  using GoalToTargetTrajectories =
      std::function<TargetTrajectories(const vector_t& goal, const SystemObservation& observation)>;

  TargetTrajectoriesGoalPublisher(::ros::NodeHandle& nh, const std::string& topic_prefix,
                                  GoalToTargetTrajectories goal_to_target_trajectories)
    : goal_to_target_trajectories_(std::move(goal_to_target_trajectories))
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
      vector_t goal_cmd = vector_t::Zero(6);
      goal_cmd[0] = msg->pose.position.x;
      goal_cmd[1] = msg->pose.position.y;
      goal_cmd[2] = msg->pose.position.z;
      Eigen::Quaternion<scalar_t> q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                                    msg->pose.orientation.z);
      goal_cmd[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
      goal_cmd[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
      goal_cmd[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

      const auto trajectories = goal_to_target_trajectories_(goal_cmd, latest_observation_);
      target_trajectories_publisher_->publishTargetTrajectories(trajectories);
    };
    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goal_callback);
  }

private:
  GoalToTargetTrajectories goal_to_target_trajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> target_trajectories_publisher_;

  ::ros::Subscriber observation_sub_, goal_sub_;
  mutable std::mutex latest_observation_mutex_;
  SystemObservation latest_observation_;
};

}  // namespace legged
