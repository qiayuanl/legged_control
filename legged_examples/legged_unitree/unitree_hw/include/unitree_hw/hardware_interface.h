
//
// Created by qiayuan on 1/24/22.
//

#pragma once

#include "unitree_legged_sdk/udp.h"
#include "unitree_legged_sdk/safety.h"

#include <legged_hw/hardware_interface.h>

namespace legged
{
const std::vector<std::string> CONTACT_SENSOR_NAMES = { "RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT" };

struct UnitreeMotorData
{
  double pos_, vel_, tau_;                   // state
  double pos_des_, vel_des_, kp_, kd_, ff_;  // command
};

struct UnitreeImuData
{
  double ori[4];
  double ori_cov[9];
  double angular_vel[3];
  double angular_vel_cov[9];
  double linear_acc[3];
  double linear_acc_cov[9];
};

class UnitreeHW : public LeggedHW
{
public:
  UnitreeHW() = default;
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
  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * Call @ref UNITREE_LEGGED_SDK::UDP::Recv() to get robot's state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref UNITREE_LEGGED_SDK::UDP::Recv(). Publish actuator
   * current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

private:
  bool setupJoints();

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle& nh);

  std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
  std::shared_ptr<UNITREE_LEGGED_SDK::Safety> safety_;
  UNITREE_LEGGED_SDK::LowState low_state_{};
  UNITREE_LEGGED_SDK::LowCmd low_cmd_{};

  UnitreeMotorData joint_data_[12]{};
  UnitreeImuData imu_data_{};
  bool contact_state_[4]{};

  int power_limit_{};
  int contact_threshold_{};
};

}  // namespace legged
