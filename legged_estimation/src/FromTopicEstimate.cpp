//
// Created by qiayuan on 2022/7/24.
//

#include "legged_estimation/FromTopiceEstimate.h"

namespace legged {
FromTopicStateEstimate::FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                               const PinocchioEndEffectorKinematics& eeKinematics,
                                               const std::vector<HybridJointHandle>& hybridJointHandles,
                                               const std::vector<ContactSensorHandle>& contactSensorHandles,
                                               const hardware_interface::ImuSensorHandle& imuSensorHandle)
    : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics, hybridJointHandles, contactSensorHandles,
                        imuSensorHandle) {
  ros::NodeHandle nh;
  sub_ = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, &FromTopicStateEstimate::callback, this);
}

void FromTopicStateEstimate::callback(const nav_msgs::Odometry::ConstPtr& msg) {
  buffer_.writeFromNonRT(*msg);
}

vector_t FromTopicStateEstimate::update(const ros::Time& time, const ros::Duration& /*period*/) {
  nav_msgs::Odometry odom = *buffer_.readFromRT();

  updateAngular(Eigen::Quaternion<scalar_t>(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                            odom.pose.pose.orientation.z),
                Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z));
  updateLinear(Eigen::Matrix<scalar_t, 3, 1>(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
               Eigen::Matrix<scalar_t, 3, 1>(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z));
  updateJointStates();

  publishMsgs(odom, time);

  return rbdState_;
}

}  // namespace legged
