/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 2/10/21.
//

#pragma once

#include <deque>
#include <unordered_map>

#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

namespace legged {
struct HybridJointData {
  hardware_interface::JointHandle joint_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};

struct HybridJointCommand {
  ros::Time stamp_;
  double posDes_{}, velDes_{}, kp_{}, kd_{}, ff_{};
};

struct ImuData {
  gazebo::physics::LinkPtr linkPtr_;
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class LeggedHWSim : public gazebo_ros_control::DefaultRobotHWSim {
 public:
  bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
               const urdf::Model* urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions) override;
  void readSim(ros::Time time, ros::Duration period) override;
  void writeSim(ros::Time time, ros::Duration period) override;

 private:
  void parseImu(XmlRpc::XmlRpcValue& imuDatas, const gazebo::physics::ModelPtr& parentModel);
  void parseContacts(XmlRpc::XmlRpcValue& contactNames);

  HybridJointInterface hybridJointInterface_;
  ContactSensorInterface contactSensorInterface_;
  hardware_interface::ImuSensorInterface imuSensorInterface_;

  gazebo::physics::ContactManager* contactManager_{};

  std::list<HybridJointData> hybridJointDatas_;
  std::list<ImuData> imuDatas_;
  std::unordered_map<std::string, std::deque<HybridJointCommand> > cmdBuffer_;
  std::unordered_map<std::string, bool> name2contact_;

  double delay_{};
};

}  // namespace legged
