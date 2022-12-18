//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/Wbc.h>

#include "legged_controllers/SafetyChecker.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class LeggedController : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
 public:
  LeggedController() = default;
  ~LeggedController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override { mpcRunning_ = false; }

 protected:
  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(LeggedInterface& legged_interface, const std::vector<HybridJointHandle>& hybrid_joint_handles,
                                  const std::vector<ContactSensorHandle>& contact_sensor_handles,
                                  const hardware_interface::ImuSensorHandle& imu_sensor_handle);

  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<Wbc> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;

  std::shared_ptr<LeggedRobotVisualizer> visualizer_;
  ros::Publisher observationPublisher_;

  SystemObservation currentObservation_;
  std::vector<HybridJointHandle> hybridJointHandles_;

 private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
};

class LeggedCheaterController : public LeggedController {
 protected:
  void setupStateEstimate(LeggedInterface& legged_interface, const std::vector<HybridJointHandle>& hybrid_joint_handles,
                          const std::vector<ContactSensorHandle>& contact_sensor_handles,
                          const hardware_interface::ImuSensorHandle& imu_sensor_handle) override;
};

}  // namespace legged
