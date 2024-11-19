
//
// Created by qiayuan on 1/24/22.
//

#pragma once

#include <legged_hw/LeggedHW.h>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

namespace UNITREE_LEGGED_SDK {
    const int FR_ = 0;
    const int FL_ = 1;
    const int RR_ = 2;
    const int RL_ = 3;
}

namespace legged {
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct UnitreeMotorData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
};

struct UnitreeImuData {
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class UnitreeHW : public LeggedHW {
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

  void updateContact(const ros::Time& time);

  uint32_t crc32_core(uint32_t* ptr, uint32_t len);

  void LowStateMessageHandler(const void* messages);

 private:
// ###############################################################################

  unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
  unitree_go::msg::dds_::LowState_ low_state{};  // default init

  unitree_go::msg::dds_::LowState_ tmp_low_state{};  // default init

  /*publisher*/
  ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
  /*subscriber*/
  ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

  // const char* NETWORK_INTERFACE = "enp59s0";
  const char* NETWORK_INTERFACE = "enp7s0";



// ###############################################################################

  bool setupJoints();

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle& nh);


  UnitreeMotorData jointData_[12]{};  // NOLINT(modernize-avoid-c-arrays)
  UnitreeImuData imuData_{};
  bool contactState_[4]{};  // NOLINT(modernize-avoid-c-arrays)

  int powerLimit_{};
  int contactThreshold_{};

  //ros::Publisher joyPublisher_;
  ros::Publisher contactPublisher_;
  ros::Time lastJoyPub_, lastContactPub_;
};

}  // namespace legged
