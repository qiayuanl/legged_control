//
// Created by qiayuan on 2021/11/5.
//
#pragma once
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace legged
{
class HybridJointHandle : public hardware_interface::JointStateHandle
{
public:
  HybridJointHandle() = default;

  HybridJointHandle(const JointStateHandle& js, double* pos_des, double* vel_des, double* kp, double* kd, double* ff)
    : JointStateHandle(js), pos_des_(pos_des), vel_des_(vel_des), kp_(kp), kd_(kd), ff_(ff)
  {
    if (!pos_des_)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Position desired data pointer is null.");
    }
    if (!vel_des_)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Velocity desired data pointer is null.");
    }
    if (!kp_)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Kp data pointer is null.");
    }
    if (!kd_)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Kd data pointer is null.");
    }
    if (!ff_)
    {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() +
                                                           "'. Feedforward data pointer is null.");
    }
  }
  void setPositionDesired(double cmd)
  {
    assert(pos_des_);
    *pos_des_ = cmd;
  }
  void setVelocityDesired(double cmd)
  {
    assert(vel_des_);
    *vel_des_ = cmd;
  }
  void setKp(double cmd)
  {
    assert(kp_);
    *kp_ = cmd;
  }
  void setKd(double cmd)
  {
    assert(kd_);
    *kd_ = cmd;
  }
  void setFeedforward(double cmd)
  {
    assert(ff_);
    *ff_ = cmd;
  }
  void setCommand(double pos_des, double vel_des, double kp, double kd, double ff)
  {
    setPositionDesired(pos_des);
    setVelocityDesired(vel_des);
    setKp(kp);
    setKd(kd);
    setFeedforward(ff);
  }
  double getPositionDesired()
  {
    assert(pos_des_);
    return *pos_des_;
  }
  double getVelocityDesired()
  {
    assert(vel_des_);
    return *vel_des_;
  }
  double getKp()
  {
    assert(kp_);
    return *kp_;
  }
  double getKd()
  {
    assert(kd_);
    return *kd_;
  }
  double getFeedforward()
  {
    assert(ff_);
    return *ff_;
  }

private:
  double* pos_des_ = { nullptr };
  double* vel_des_ = { nullptr };
  double* kp_ = { nullptr };
  double* kd_ = { nullptr };
  double* ff_ = { nullptr };
};

class HybridJointInterface
  : public hardware_interface::HardwareResourceManager<HybridJointHandle, hardware_interface::ClaimResources>
{
};

}  // namespace legged
