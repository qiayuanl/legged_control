#pragma once

#include <legged_hw/LeggedHW.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

#include <string>
#include <vector>

namespace legged {

const std::vector<std::string> ISAAC_CONTACT_SENSOR_NAMES = {
    "LF_FOOT",
    "LH_FOOT",
    "RF_FOOT",
    "RH_FOOT"
};
struct IsaacMotorData {
  double pos_{0.0}, vel_{0.0}, tau_{0.0};                 // measured state
  double posDes_{0.0}, velDes_{0.0}, kp_{0.0}, kd_{0.0}, ff_{0.0};  // command
};

struct IsaacImuData {
  double ori_[4] = {0.0, 0.0, 0.0, 1.0};
  double oriCov_[9] = {0.0};
  double angularVel_[3] = {0.0};
  double angularVelCov_[9] = {0.0};
  double linearAcc_[3] = {0.0};
  double linearAccCov_[9] = {0.0};
};

class IsaacHW : public LeggedHW {
 public:
  IsaacHW() = default;

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

 private:
  bool setupJoints();
  bool setupImu();
  bool setupContactSensor(ros::NodeHandle& nh);

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void contactCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

  IsaacMotorData jointData_[12]{};
  IsaacImuData imuData_{};
  bool contactState_[4]{};

  sensor_msgs::JointState latestJointState_;
  sensor_msgs::Imu latestImu_;
  nav_msgs::Odometry latestOdom_;

  bool jointStateReceived_ = false;
  bool imuReceived_ = false;
  bool odomReceived_ = false;
  bool contactReceived_ = false;

  ros::Subscriber jointStateSub_;
  ros::Subscriber imuSub_;
  ros::Subscriber odomSub_;
  ros::Subscriber contactSub_;

  ros::Publisher jointCmdPub_;
  ros::Publisher groundTruthPub_;
};

}  // namespace legged