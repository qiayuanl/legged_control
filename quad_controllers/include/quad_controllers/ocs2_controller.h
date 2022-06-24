//
// Created by qiayuan on 2022/6/24.
//

#pragma once
#include <controller_interface/multi_interface_controller.h>
#include <quad_common/hardware_interface/hybrid_joint_interface.h>
#include <quad_common/hardware_interface/contact_sensor_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_sqp/MultipleShootingMpc.h>

namespace quad_ros
{
using namespace ocs2;
using namespace legged_robot;

class Ocs2Controller
  : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                          ContactSensorInterface>
{
public:
  Ocs2Controller() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

protected:
  std::shared_ptr<LeggedRobotInterface> legged_interface_;
  std::shared_ptr<MultipleShootingMpc> mpc_;
};

}  // namespace quad_ros
