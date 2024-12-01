//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>

#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class StandUpController : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
 public:
  StandUpController() = default;
  ~StandUpController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override { }

 protected:
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  // Interface
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

 private:
  std::atomic_bool controllerRunning_{};

  float _targetPos_1[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                            -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

  float _targetPos_2[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                            0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

  float _targetPos_3[12] = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                            -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};

  float _startPos[12];
  float _duration_1 = 500;   
  float _duration_2 = 500; 
  float _duration_3 = 1000;   
  float _duration_4 = 900;   
  float _percent_1 = 0;    
  float _percent_2 = 0;    
  float _percent_3 = 0;    
  float _percent_4 = 0;    

  bool firstRun = true;
  bool done = false;

};



}  // namespace legged
