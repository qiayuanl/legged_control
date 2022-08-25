
//
// Created by qiayuan on 1/24/22.
//

#include "unitree_hw/hardware_interface.h"

namespace legged
{
bool UnitreeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  if (!LeggedHW::init(root_nh, robot_hw_nh))
    return false;

  robot_hw_nh.getParam("power_limit", power_limit_);

  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);

  udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
  udp_->InitCmdData(low_cmd_);

  std::string robot_type;
  root_nh.getParam("robot_type", robot_type);
  if (robot_type == "a1")
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::A1);
  else if (robot_type == "aliengo")
    safety_ = std::make_shared<UNITREE_LEGGED_SDK::Safety>(UNITREE_LEGGED_SDK::LeggedType::Aliengo);
  else
  {
    ROS_FATAL("Unknown robot type: %s", robot_type.c_str());
    return false;
  }
  return true;
}

void UnitreeHW::read(const ros::Time& time, const ros::Duration& period)
{
  udp_->Recv();
  udp_->GetRecv(low_state_);

  for (int i = 0; i < 12; ++i)
  {
    joint_data_[i].pos_ = low_state_.motorState[i].q;
    joint_data_[i].vel_ = low_state_.motorState[i].dq;
    joint_data_[i].tau_ = low_state_.motorState[i].tauEst;
  }

  imu_data_.ori[0] = low_state_.imu.quaternion[1];
  imu_data_.ori[1] = low_state_.imu.quaternion[2];
  imu_data_.ori[2] = low_state_.imu.quaternion[3];
  imu_data_.ori[3] = low_state_.imu.quaternion[0];
  imu_data_.angular_vel[0] = low_state_.imu.gyroscope[0];
  imu_data_.angular_vel[1] = low_state_.imu.gyroscope[1];
  imu_data_.angular_vel[2] = low_state_.imu.gyroscope[2];
  imu_data_.linear_acc[0] = low_state_.imu.accelerometer[0];
  imu_data_.linear_acc[1] = low_state_.imu.accelerometer[1];
  imu_data_.linear_acc[2] = low_state_.imu.accelerometer[2];

  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i)
    contact_state_[i] = low_state_.footForce[i] > contact_threshold_;

  // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
  std::vector<std::string> names = hybrid_joint_interface_.getNames();
  for (const auto& name : names)
  {
    HybridJointHandle handle = hybrid_joint_interface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(3.);
  }
}

void UnitreeHW::write(const ros::Time& time, const ros::Duration& period)
{
  for (int i = 0; i < 12; ++i)
  {
    low_cmd_.motorCmd[i].q = joint_data_[i].pos_des_;
    low_cmd_.motorCmd[i].dq = joint_data_[i].vel_des_;
    low_cmd_.motorCmd[i].Kp = joint_data_[i].kp_;
    low_cmd_.motorCmd[i].Kd = joint_data_[i].kd_;
    low_cmd_.motorCmd[i].tau = joint_data_[i].ff_;
  }
  safety_->PositionLimit(low_cmd_);
  safety_->PowerProtect(low_cmd_, low_state_, power_limit_);
  udp_->SetSend(low_cmd_);
  udp_->Send();
}

bool UnitreeHW::setupJoints()
{
  for (const auto& joint : urdf_model_->joints_)
  {
    int leg_index, joint_index;
    if (joint.first.find("RF") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::FR_;
    else if (joint.first.find("LF") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::FL_;
    else if (joint.first.find("RH") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::RR_;
    else if (joint.first.find("LH") != std::string::npos)
      leg_index = UNITREE_LEGGED_SDK::RL_;
    else
      continue;
    if (joint.first.find("HAA") != std::string::npos)
      joint_index = 0;
    else if (joint.first.find("HFE") != std::string::npos)
      joint_index = 1;
    else if (joint.first.find("KFE") != std::string::npos)
      joint_index = 2;
    else
      continue;

    int index = leg_index * 3 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &joint_data_[index].pos_, &joint_data_[index].vel_,
                                                      &joint_data_[index].tau_);
    joint_state_interface_.registerHandle(state_handle);
    hybrid_joint_interface_.registerHandle(HybridJointHandle(state_handle, &joint_data_[index].pos_des_,
                                                             &joint_data_[index].vel_des_, &joint_data_[index].kp_,
                                                             &joint_data_[index].kd_, &joint_data_[index].ff_));
  }
  return true;
}

bool UnitreeHW::setupImu()
{
  imu_sensor_interface_.registerHandle(hardware_interface::ImuSensorHandle(
      "unitree_imu", "unitree_imu", imu_data_.ori, imu_data_.ori_cov, imu_data_.angular_vel, imu_data_.angular_vel_cov,
      imu_data_.linear_acc, imu_data_.linear_acc_cov));
  imu_data_.ori_cov[0] = 0.0012;
  imu_data_.ori_cov[4] = 0.0012;
  imu_data_.ori_cov[8] = 0.0012;

  imu_data_.angular_vel_cov[0] = 0.0004;
  imu_data_.angular_vel_cov[4] = 0.0004;
  imu_data_.angular_vel_cov[8] = 0.0004;

  return true;
}

bool UnitreeHW::setupContactSensor(ros::NodeHandle& nh)
{
  nh.getParam("contact_threshold", contact_threshold_);
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i)
    contact_sensor_interface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contact_state_[i]));
  return true;
}

}  // namespace legged
