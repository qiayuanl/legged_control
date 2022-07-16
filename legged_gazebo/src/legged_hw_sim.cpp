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

#include "legged_gazebo/legged_hw_sim.h"
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>

namespace legged
{
bool QuadHWSim::initSim(const std::string& robot_namespace, ros::NodeHandle model_nh,
                        gazebo::physics::ModelPtr parent_model, const urdf::Model* urdf_model,
                        std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  bool ret = DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);
  // Joint interface
  registerInterface(&hybrid_joint_interface_);
  std::vector<std::string> names = ej_interface_.getNames();
  for (const auto& name : names)
  {
    hybrid_joint_datas_.push_back(HybridJointData{ .joint_ = ej_interface_.getHandle(name) });
    HybridJointData& back = hybrid_joint_datas_.back();
    hybrid_joint_interface_.registerHandle(
        HybridJointHandle(back.joint_, &back.pos_des_, &back.vel_des_, &back.kp_, &back.kd_, &back.ff_));
    cmd_buffer_.insert(
        std::make_pair<std::string, std::deque<HybridJointCommand>>(name.c_str(), std::deque<HybridJointCommand>()));
  }
  // IMU interface
  registerInterface(&imu_sensor_interface_);
  XmlRpc::XmlRpcValue xml_rpc_value;
  if (!model_nh.getParam("gazebo/imus", xml_rpc_value))
    ROS_WARN("No imu specified");
  else
    parseImu(xml_rpc_value, parent_model);
  if (!model_nh.getParam("gazebo/delay", delay_))
    delay_ = 0.;
  if (!model_nh.getParam("gazebo/contacts", xml_rpc_value))
    ROS_WARN("No contacts specified");
  else
    parseContacts(xml_rpc_value);

  contact_manager_ = parent_model->GetWorld()->Physics()->GetContactManager();
  contact_manager_->SetNeverDropContacts(true);  // NOTE: If false, we need to select view->contacts in gazebo GUI to
                                                 // avoid returning nothing when calling ContactManager::GetContacts()
  return ret;
}

void QuadHWSim::readSim(ros::Time time, ros::Duration period)
{
  gazebo_ros_control::DefaultRobotHWSim::readSim(time, period);
  // Imu Sensor
  for (auto& imu : imu_datas_)
  {
    // TODO(qiayuan) Add noise
    ignition::math::Pose3d pose = imu.link_prt->WorldPose();
    imu.ori[0] = pose.Rot().X();
    imu.ori[1] = pose.Rot().Y();
    imu.ori[2] = pose.Rot().Z();
    imu.ori[3] = pose.Rot().W();
    ignition::math::Vector3d rate = imu.link_prt->RelativeAngularVel();
    imu.angular_vel[0] = rate.X();
    imu.angular_vel[1] = rate.Y();
    imu.angular_vel[2] = rate.Z();

    ignition::math::Vector3d gravity = { 0., 0., -9.81 };
    ignition::math::Vector3d accel = imu.link_prt->RelativeLinearAccel() - pose.Rot().RotateVectorReverse(gravity);
    imu.linear_acc[0] = accel.X();
    imu.linear_acc[1] = accel.Y();
    imu.linear_acc[2] = accel.Z();
  }

  // Contact Sensor
  for (auto& state : name2contact_)
    state.second = false;
  for (const auto& contact : contact_manager_->GetContacts())
  {
    if (static_cast<uint32_t>(contact->time.sec) != time.sec ||
        static_cast<uint32_t>(contact->time.nsec) != (time - period).nsec)
      continue;
    std::string link_name = contact->collision1->GetLink()->GetName();
    if (name2contact_.find(link_name) != name2contact_.end())
      name2contact_[link_name] = true;
    link_name = contact->collision2->GetLink()->GetName();
    if (name2contact_.find(link_name) != name2contact_.end())
      name2contact_[link_name] = true;
  }

  // Set cmd to zero to avoid crazy soft limit oscillation when not controller loaded
  for (auto& cmd : joint_effort_command_)
    cmd = 0;
  for (auto& cmd : joint_velocity_command_)
    cmd = 0;
}

void QuadHWSim::writeSim(ros::Time time, ros::Duration period)
{
  for (auto joint : hybrid_joint_datas_)
  {
    auto& buffer = cmd_buffer_.find(joint.joint_.getName())->second;
    buffer.push_front(HybridJointCommand{ .stamp_ = time,
                                          .pos_des_ = joint.pos_des_,
                                          .vel_des_ = joint.vel_des_,
                                          .kp_ = joint.kp_,
                                          .kd_ = joint.kd_,
                                          .ff_ = joint.ff_ });

    while (!buffer.empty() && buffer.back().stamp_ + ros::Duration(delay_) < time)
      buffer.pop_back();
    const auto& cmd = buffer.back();
    joint.joint_.setCommand(cmd.kp_ * (cmd.pos_des_ - joint.joint_.getPosition()) +
                            cmd.kd_ * (cmd.vel_des_ - joint.joint_.getVelocity()) + cmd.ff_);
  }
  DefaultRobotHWSim::writeSim(time, period);
}

void QuadHWSim::parseImu(XmlRpc::XmlRpcValue& imu_datas, const gazebo::physics::ModelPtr& parent_model)
{
  ROS_ASSERT(imu_datas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (auto it = imu_datas.begin(); it != imu_datas.end(); ++it)
  {
    if (!it->second.hasMember("frame_id"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated frame id.");
      continue;
    }
    else if (!it->second.hasMember("orientation_covariance_diagonal"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated orientation covariance diagonal.");
      continue;
    }
    else if (!it->second.hasMember("angular_velocity_covariance"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated angular velocity covariance.");
      continue;
    }
    else if (!it->second.hasMember("linear_acceleration_covariance"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated linear acceleration covariance.");
      continue;
    }
    XmlRpc::XmlRpcValue ori_cov = imu_datas[it->first]["orientation_covariance_diagonal"];
    ROS_ASSERT(ori_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(ori_cov.size() == 3);
    for (int i = 0; i < ori_cov.size(); ++i)
      ROS_ASSERT(ori_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    XmlRpc::XmlRpcValue angular_cov = imu_datas[it->first]["orientation_covariance_diagonal"];
    ROS_ASSERT(angular_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(angular_cov.size() == 3);
    for (int i = 0; i < angular_cov.size(); ++i)
      ROS_ASSERT(angular_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    XmlRpc::XmlRpcValue linear_cov = imu_datas[it->first]["linear_acceleration_covariance"];
    ROS_ASSERT(linear_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(linear_cov.size() == 3);
    for (int i = 0; i < linear_cov.size(); ++i)
      ROS_ASSERT(linear_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    std::string frame_id = imu_datas[it->first]["frame_id"];
    gazebo::physics::LinkPtr link_ptr = parent_model->GetLink(frame_id);
    ROS_ASSERT(link_ptr != nullptr);
    imu_datas_.push_back((ImuData{
        .link_prt = link_ptr,
        .ori = { 0., 0., 0., 0. },
        .ori_cov = { static_cast<double>(ori_cov[0]), 0., 0., 0., static_cast<double>(ori_cov[1]), 0., 0., 0.,
                     static_cast<double>(ori_cov[2]) },
        .angular_vel = { 0., 0., 0. },
        .angular_vel_cov = { static_cast<double>(angular_cov[0]), 0., 0., 0., static_cast<double>(angular_cov[1]), 0.,
                             0., 0., static_cast<double>(angular_cov[2]) },
        .linear_acc = { 0., 0., 0. },
        .linear_acc_cov = { static_cast<double>(linear_cov[0]), 0., 0., 0., static_cast<double>(linear_cov[1]), 0., 0.,
                            0., static_cast<double>(linear_cov[2]) } }));
    ImuData& imu_data = imu_datas_.back();
    imu_sensor_interface_.registerHandle(
        hardware_interface::ImuSensorHandle(it->first, frame_id, imu_data.ori, imu_data.ori_cov, imu_data.angular_vel,
                                            imu_data.angular_vel_cov, imu_data.linear_acc, imu_data.linear_acc_cov));
  }
}

void QuadHWSim::parseContacts(XmlRpc::XmlRpcValue& contact_names)
{
  ROS_ASSERT(contact_names.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < contact_names.size(); ++i)
  {
    std::string name = contact_names[i];
    name2contact_.insert(std::make_pair(name, false));
    contact_sensor_interface_.registerHandle(ContactSensorHandle(name, &name2contact_[name]));
  }
  registerInterface(&contact_sensor_interface_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::QuadHWSim, gazebo_ros_control::RobotHWSim)
GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::GazeboRosControlPlugin)  // Default plugin
