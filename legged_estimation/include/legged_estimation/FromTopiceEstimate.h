//
// Created by qiayuan on 2022/7/24.
//

#include "legged_estimation/StateEstimateBase.h"

#include <realtime_tools/realtime_buffer.h>

#pragma once
namespace legged {
using namespace ocs2;

class FromTopicStateEstimate : public StateEstimateBase {
 public:
  FromTopicStateEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                         const PinocchioEndEffectorKinematics& eeKinematics, const std::vector<HybridJointHandle>& hybridJointHandles,
                         const std::vector<ContactSensorHandle>& contactSensorHandles,
                         const hardware_interface::ImuSensorHandle& imuSensorHandle);

  vector_t update(const ros::Time& time, const ros::Duration& period) override;

 private:
  void callback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Subscriber sub_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> buffer_;
};

}  // namespace legged
