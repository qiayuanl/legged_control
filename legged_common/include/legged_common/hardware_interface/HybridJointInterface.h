//
// Created by qiayuan on 2021/11/5.
//
#pragma once
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace legged {
class HybridJointHandle : public hardware_interface::JointStateHandle {
 public:
  HybridJointHandle() = default;

  HybridJointHandle(const JointStateHandle& js, double* posDes, double* velDes, double* kp, double* kd, double* ff)
      : JointStateHandle(js), posDes_(posDes), velDes_(velDes), kp_(kp), kd_(kd), ff_(ff) {
    if (posDes_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Position desired data pointer is null.");
    }
    if (velDes_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Velocity desired data pointer is null.");
    }
    if (kp_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Kp data pointer is null.");
    }
    if (kd_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Kd data pointer is null.");
    }
    if (ff_ == nullptr) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Feedforward data pointer is null.");
    }
  }
  void setPositionDesired(double cmd) {
    assert(posDes_);
    *posDes_ = cmd;
  }
  void setVelocityDesired(double cmd) {
    assert(velDes_);
    *velDes_ = cmd;
  }
  void setKp(double cmd) {
    assert(kp_);
    *kp_ = cmd;
  }
  void setKd(double cmd) {
    assert(kd_);
    *kd_ = cmd;
  }
  void setFeedforward(double cmd) {
    assert(ff_);
    *ff_ = cmd;
  }
  void setCommand(double pos_des, double vel_des, double kp, double kd, double ff) {
    setPositionDesired(pos_des);
    setVelocityDesired(vel_des);
    setKp(kp);
    setKd(kd);
    setFeedforward(ff);
  }
  double getPositionDesired() {
    assert(posDes_);
    return *posDes_;
  }
  double getVelocityDesired() {
    assert(velDes_);
    return *velDes_;
  }
  double getKp() {
    assert(kp_);
    return *kp_;
  }
  double getKd() {
    assert(kd_);
    return *kd_;
  }
  double getFeedforward() {
    assert(ff_);
    return *ff_;
  }

 private:
  double* posDes_ = {nullptr};
  double* velDes_ = {nullptr};
  double* kp_ = {nullptr};
  double* kd_ = {nullptr};
  double* ff_ = {nullptr};
};

class HybridJointInterface : public hardware_interface::HardwareResourceManager<HybridJointHandle, hardware_interface::ClaimResources> {};

}  // namespace legged
