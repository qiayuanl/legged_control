//
// Created by qiayuan on 2022/6/24.
//

#pragma once
#include "quad_controllers/state_estimate.h"

#include <controller_interface/multi_interface_controller.h>
#include <quad_common/hardware_interface/contact_sensor_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

#include <ocs2_sqp/MultipleShootingMpc.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <quad_wbc/wbc.h>

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
  ~Ocs2Controller() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override
  {
    mpc_running_ = false;
  }

protected:
  std::shared_ptr<LeggedRobotInterface> legged_interface_;
  std::shared_ptr<MultipleShootingMpc> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpc_mrt_interface_;
  std::shared_ptr<CentroidalModelRbdConversions> rbd_conversions_;
  std::shared_ptr<StateEstimateBase> state_estimate_;
  std::shared_ptr<Wbc> wbc_;

  std::shared_ptr<LeggedRobotVisualizer> visualizer_;
  ros::Publisher observation_publisher_;

  SystemObservation current_observation_;
  std::vector<HybridJointHandle> hybrid_joint_handles_;

private:
  std::thread mpc_thread_;
  std::atomic_bool controller_running_, mpc_running_{};
};

}  // namespace quad_ros
