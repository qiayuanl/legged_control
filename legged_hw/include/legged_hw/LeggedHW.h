
//
// Created by qiayuan on 6/24/22.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>

// ROS control
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

namespace legged {
class LeggedHW : public hardware_interface::RobotHW {
 public:
  LeggedHW() = default;
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

 protected:
  // Interface
  hardware_interface::JointStateInterface jointStateInterface_;  // NOLINT(misc-non-private-member-variables-in-classes)
  hardware_interface::ImuSensorInterface imuSensorInterface_;    // NOLINT(misc-non-private-member-variables-in-classes)
  HybridJointInterface hybridJointInterface_;                    // NOLINT(misc-non-private-member-variables-in-classes)
  ContactSensorInterface contactSensorInterface_;                // NOLINT(misc-non-private-member-variables-in-classes)
  // URDF model of the robot
  std::shared_ptr<urdf::Model> urdfModel_;  // NOLINT(misc-non-private-member-variables-in-classes)

 private:
  /** \brief Load urdf of robot from param server.
   *
   * Load urdf of robot from param server.
   *
   * @param rootNh Root node-handle of a ROS node
   * @return True if successful.
   */
  bool loadUrdf(ros::NodeHandle& rootNh);
};

}  // namespace legged
