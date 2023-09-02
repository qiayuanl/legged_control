//
// Created by luohx on 23-8-25.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <gazebo_msgs/ModelStates.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <std_msgs/Float32MultiArray.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <Eigen/Geometry>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

struct RLRobotCfg {
  struct ControlCfg {
    float stiffness;
    float damping;
    float actionScale;
    int decimation;
  };

  struct InitState {
    // default joint angles
    scalar_t LF_HAA_joint;
    scalar_t LF_HFE_joint;
    scalar_t LF_KFE_joint;

    scalar_t LH_HAA_joint;
    scalar_t LH_HFE_joint;
    scalar_t LH_KFE_joint;

    scalar_t RF_HAA_joint;
    scalar_t RF_HFE_joint;
    scalar_t RF_KFE_joint;

    scalar_t RH_HAA_joint;
    scalar_t RH_HFE_joint;
    scalar_t RH_KFE_joint;
  };

  struct ObsScales {
    scalar_t linVel;
    scalar_t angVel;
    scalar_t dofPos;
    scalar_t dofVel;
    scalar_t heightMeasurements;
  };

  scalar_t clipActions;
  scalar_t clipObs;

  InitState initState;
  ObsScales obsScales;
  ControlCfg controlCfg;
};

class RLControllerBase : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
 public:
  enum class Mode : uint8_t { LIE, STAND, WALK };

  RLControllerBase() = default;
  virtual ~RLControllerBase() = default;
  virtual bool init(hardware_interface::RobotHW* robotHw, ros::NodeHandle& controllerNH);
  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& period);

  virtual bool loadModel(ros::NodeHandle& nh) { return false; };
  virtual bool loadRLCfg(ros::NodeHandle& nh) { return false; };
  virtual void computeActions(){};
  virtual void computeObservation(){};

  virtual void handleLieMode();
  virtual void handleStandMode();
  virtual void handleWalkMode(){};

 protected:
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);
  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);

  void cmdVelCallback(const geometry_msgs::Twist& msg);

  Mode mode_;
  int64_t loopCount_;
  vector3_t command_;

  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

  vector_t rbdState_;
  vector_t measuredRbdState_;
  SystemObservation currentObservation_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

  // hardware interface
  std::vector<HybridJointHandle> hybridJointHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandles_;
  std::vector<ContactSensorHandle> contactHandles_;

 private:
  // PD stand
  std::vector<scalar_t> initJointAngles_;
  vector_t standJointAngles_;

  scalar_t standPercent_;
  scalar_t standDuration_;

  ros::Subscriber cmdVelSub_;
};
}  // namespace legged
