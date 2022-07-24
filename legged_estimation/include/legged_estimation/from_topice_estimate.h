//
// Created by qiayuan on 2022/7/24.
//

#include "legged_estimation/state_estimate_base.h"

#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>

#pragma once
namespace legged
{
using namespace ocs2;

class FromTopicStateEstimate : public StateEstimateBase
{
public:
  FromTopicStateEstimate(LeggedInterface& legged_interface, const std::vector<HybridJointHandle>& hybrid_joint_handles,
                         const std::vector<ContactSensorHandle>& contact_sensor_handles,
                         const hardware_interface::ImuSensorHandle& imu_sensor_handle);

  vector_t update(scalar_t dt) override;

private:
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
};

}  // namespace legged