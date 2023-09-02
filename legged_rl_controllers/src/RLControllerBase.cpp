//
// Created by luohx on 23-8-25.
//

#include "legged_rl_controllers/RLControllerBase.h"

#include <pluginlib/class_list_macros.hpp>

namespace legged {
bool RLControllerBase::init(hardware_interface::RobotHW* robotHw, ros::NodeHandle& controllerNH) {
  // Get config file and setup legged interface
  std::string taskFile;
  std::string urdfFile;
  std::string referenceFile;
  controllerNH.getParam("/urdfFile", urdfFile);
  controllerNH.getParam("/taskFile", taskFile);
  controllerNH.getParam("/referenceFile", referenceFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());
  standJointAngles_.resize(leggedInterface_->getCentroidalModelInfo().actuatedDofNum);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", standJointAngles_);

  // Load policy model and rl cfg
  if (!loadModel(controllerNH)) {
    ROS_ERROR_STREAM("[RLControllerBase] Failed to load the model. Ensure the path is correct and accessible.");
    return false;
  }
  if (!loadRLCfg(controllerNH)) {
    ROS_ERROR_STREAM("[RLControllerBase] Failed to load the rl config. Ensure the yaml is correct and accessible.");
    return false;
  }

  // Hardware interface
  auto* hybridJointInterface = robotHw->get<HybridJointInterface>();
  std::vector<std::string> jointNames{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                         "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};

  for (const auto& jointName : jointNames) {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(jointName));
  }
  imuSensorHandles_ = robotHw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu");

  auto* contactInterface = robotHw->get<ContactSensorInterface>();
  const auto& footNames = leggedInterface_->modelSettings().contactNames3DoF;
  for (const auto& footName : footNames) {
    contactHandles_.push_back(contactInterface->getHandle(footName));
  }

  // State estimate
  setupStateEstimate(taskFile, verbose);

  cmdVelSub_ = controllerNH.subscribe("/cmd_vel", 1, &RLControllerBase::cmdVelCallback, this);

  return true;
}

void RLControllerBase::starting(const ros::Time& time) {
  updateStateEstimation(time, ros::Duration(0.002));

  for (auto& hybridJointHandle : hybridJointHandles_) {
    initJointAngles_.push_back(hybridJointHandle.getPosition());
  }

  scalar_t durationSecs = 2.0;
  standDuration_ = durationSecs * 1000.0;
  standPercent_ += 1 / standDuration_;
  mode_ = Mode::LIE;
  loopCount_ = 0;
}

void RLControllerBase::update(const ros::Time& time, const ros::Duration& period) {
  updateStateEstimation(time, period);

  switch (mode_) {
    case Mode::LIE:
      handleLieMode();
      break;
    case Mode::STAND:
      handleStandMode();
      break;
    case Mode::WALK:
      handleWalkMode();
      break;
    default:
      ROS_ERROR_STREAM("Unexpected mode encountered: " << static_cast<int>(mode_));
      break;
  }

  loopCount_++;
}

void RLControllerBase::handleLieMode() {
  if (standPercent_ < 1) {
    for (int j = 0; j < hybridJointHandles_.size(); j++) {
      scalar_t pos_des = initJointAngles_[j] * (1 - standPercent_) + standJointAngles_[j] * standPercent_;
      hybridJointHandles_[j].setCommand(pos_des, 0, 30, 0.5, 0);
    }
    standPercent_ += 1 / standDuration_;
  } else {
    mode_ = Mode::STAND;
  }
}

void RLControllerBase::handleStandMode() {
  if (loopCount_ > 3000) {
    mode_ = Mode::WALK;
  }
}

void RLControllerBase::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void RLControllerBase::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
}

void RLControllerBase::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }
  for (size_t i = 0; i < contacts.size(); ++i) {
    contactFlag[i] = contactHandles_[i].isContact();
  }
  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = imuSensorHandles_.getOrientation()[i];
  }
  for (size_t i = 0; i < 3; ++i) {
    angularVel(i) = imuSensorHandles_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandles_.getLinearAcceleration()[i];
  }
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = imuSensorHandles_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandles_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandles_.getLinearAccelerationCovariance()[i];
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  rbdState_ = stateEstimate_->update(time, period);
}

void RLControllerBase::cmdVelCallback(const geometry_msgs::Twist& msg) {
  command_(0) = msg.linear.x;
  command_(1) = msg.linear.y;
  command_(2) = msg.angular.z;
}
}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::RLControllerBase, controller_interface::ControllerBase)
