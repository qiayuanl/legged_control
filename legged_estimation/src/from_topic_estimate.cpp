//
// Created by qiayuan on 2022/7/24.
//

#include "legged_estimation/from_topice_estimate.h"

namespace legged
{
FromTopicStateEstimate::FromTopicStateEstimate(LeggedInterface& legged_interface,
                                               const std::vector<HybridJointHandle>& hybrid_joint_handles_,
                                               const std::vector<ContactSensorHandle>& contact_sensor_handles,
                                               const hardware_interface::ImuSensorHandle& imu_sensor_handle)
  : StateEstimateBase(legged_interface, hybrid_joint_handles_, contact_sensor_handles, imu_sensor_handle)
{
  ros::NodeHandle nh;
  sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 100, &FromTopicStateEstimate::callback, this);
}

void FromTopicStateEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  buffer_.writeFromNonRT(*msg);
}

vector_t FromTopicStateEstimate::update(scalar_t dt)
{
  nav_msgs::Odometry odom = *buffer_.readFromRT();

  updateAngular(Eigen::Quaternion<scalar_t>(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z),
                Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.angular.x, odom.twist.twist.angular.y,
                                              odom.twist.twist.angular.z));
  updateLinear(
      Eigen::Matrix<scalar_t, 3, 1>(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
      Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z));
  updateJointStates();

  return rbd_state_;
}

}  // namespace legged
