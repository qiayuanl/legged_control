//
// Isaac backend modeled after Gazebo LeggedHWSim behavior
// FIXED VERSION FOR PYTHON GAZEBO-STYLE TORQUE BRIDGE
//
// Responsibility split:
//
// Python bridge:
//   - Publishes backend Isaac topics:
//       /isaac/joint_states
//       /isaac/imu
//       /isaac/odom
//       /isaac/contacts
//   - Receives /isaac/joint_cmd.
//   - Applies command to Isaac.
//
// C++ IsaacHW:
//   - Subscribes to backend Isaac topics.
//   - Exposes HybridJointInterface to the controller.
//   - Publishes [q_des, dq_des, kp, kd, ff] x 12 to /isaac/joint_cmd.
//   - Republishes public Gazebo-style ground-truth odom only:
//       /ground_truth/state  frame_id = world, child_frame_id = base
//   - Does NOT publish /odom here.
//   - Does NOT publish odom -> base TF here.
//     Reason: the base legged stack/controller already interacts with odom/TF internally.
//     Publishing /odom/TF again from this backend can create mixed frames or duplicate odom data.
//   - Does NOT compute final torque.
//   - Does NOT apply stand correction.
//   - Does NOT force high gains.
//   - Does NOT command a fallback stand pose when controller is inactive.
//

#include "isaac_legged_hw/IsaacHW.h"

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

constexpr int NUM_JOINTS = 12;
constexpr int NUM_LEGS = 4;

// File-scope publisher, so IsaacHW.h does NOT need odomPub_.
// /odom is intentionally not published by this backend.

// Controller / OCS2 order:
// [LF_HAA, LF_HFE, LF_KFE,
//  LH_HAA, LH_HFE, LH_KFE,
//  RF_HAA, RF_HFE, RF_KFE,
//  RH_HAA, RH_HFE, RH_KFE]

// Used only as safe internal initialization before real /isaac/joint_states arrives.
// This is NOT published as an automatic stand command.
const std::array<double, NUM_JOINTS> GAZEBO_STAND_Q = {
    0.0794857511, 0.7901606123, -1.6145155326,
    0.0721311389, 0.7799202724, -1.6157185682,
   -0.0761236063, 0.7883236768, -1.6167676846,
   -0.0692159051, 0.7817057874, -1.6177661831,
};

// IMPORTANT:
// Keep false. We do not want IsaacHW to generate a stand command by itself.
constexpr bool USE_SAFE_STAND_FALLBACK_WHEN_NO_COMMAND = false;

// Do not force gains here.
// The controller should decide kp/kd.
// Python bridge can decide how to apply/convert them.
constexpr double MIN_ISAAC_KP = 0.0;
constexpr double MIN_ISAAC_KD = 0.0;

// Safety clipping before sending to Python.
constexpr double MAX_ISAAC_KP = 45.0;
constexpr double MAX_ISAAC_KD = 10.0;
constexpr double MAX_ISAAC_FF = 45.0;

// No q_des smoothing here.
constexpr double Q_DES_ALPHA = 1.0;

// Mild ff smoothing.
constexpr double FF_ALPHA = 0.35;
constexpr double MAX_FF_STEP = 2.0;

constexpr double COMMAND_EPS = 1e-6;

// Critical:
// Disable old q_des correction. Python/Isaac should receive controller targets.
constexpr bool ENABLE_ISAAC_STAND_CONVENTION_CORRECTION = false;

inline double safeValue(double value, double fallback = 0.0) {
  return std::isfinite(value) ? value : fallback;
}

inline double clampValue(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

inline double rateLimit(double target, double previous, double maxStep) {
  const double diff = target - previous;

  if (diff > maxStep) {
    return previous + maxStep;
  }

  if (diff < -maxStep) {
    return previous - maxStep;
  }

  return target;
}

inline int isaacJointNameToIndex(const std::string& name) {
  static const std::unordered_map<std::string, int> kMap = {
      // OCS2 / Gazebo names
      {"LF_HAA", 0}, {"LF_HFE", 1}, {"LF_KFE", 2},
      {"LH_HAA", 3}, {"LH_HFE", 4}, {"LH_KFE", 5},
      {"RF_HAA", 6}, {"RF_HFE", 7}, {"RF_KFE", 8},
      {"RH_HAA", 9}, {"RH_HFE", 10}, {"RH_KFE", 11},

      // Isaac / Unitree USD names
      {"FL_hip_joint", 0}, {"FL_thigh_joint", 1}, {"FL_calf_joint", 2},
      {"RL_hip_joint", 3}, {"RL_thigh_joint", 4}, {"RL_calf_joint", 5},
      {"FR_hip_joint", 6}, {"FR_thigh_joint", 7}, {"FR_calf_joint", 8},
      {"RR_hip_joint", 9}, {"RR_thigh_joint", 10}, {"RR_calf_joint", 11},
  };

  const auto it = kMap.find(name);
  return (it == kMap.end()) ? -1 : it->second;
}

inline int urdfJointNameToIndex(const std::string& name) {
  int legIndex = -1;
  int jointIndex = -1;

  if (name.find("LF") != std::string::npos ||
      name.find("FL") != std::string::npos) {
    legIndex = 0;
  } else if (name.find("LH") != std::string::npos ||
             name.find("RL") != std::string::npos) {
    legIndex = 1;
  } else if (name.find("RF") != std::string::npos ||
             name.find("FR") != std::string::npos) {
    legIndex = 2;
  } else if (name.find("RH") != std::string::npos ||
             name.find("RR") != std::string::npos) {
    legIndex = 3;
  } else {
    return -1;
  }

  if (name.find("HAA") != std::string::npos ||
      name.find("hip") != std::string::npos) {
    jointIndex = 0;
  } else if (name.find("HFE") != std::string::npos ||
             name.find("thigh") != std::string::npos) {
    jointIndex = 1;
  } else if (name.find("KFE") != std::string::npos ||
             name.find("calf") != std::string::npos) {
    jointIndex = 2;
  } else {
    return -1;
  }

  return legIndex * 3 + jointIndex;
}

inline bool hasRealControllerCommand(
    const std::array<double, NUM_JOINTS>& qDes,
    const std::array<double, NUM_JOINTS>& qNow,
    const std::array<double, NUM_JOINTS>& kp,
    const std::array<double, NUM_JOINTS>& kd,
    const std::array<double, NUM_JOINTS>& ff) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (std::abs(kp[i]) > COMMAND_EPS ||
        std::abs(kd[i]) > COMMAND_EPS ||
        std::abs(ff[i]) > COMMAND_EPS) {
      return true;
    }

    if (std::abs(qDes[i] - qNow[i]) > 1e-4) {
      return true;
    }
  }

  return false;
}

}  // namespace


namespace legged {

bool IsaacHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    ROS_ERROR("Failed to initialize base LeggedHW.");
    return false;
  }

  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);

  jointStateSub_ = root_nh.subscribe(
      "/isaac/joint_states",
      1,
      &IsaacHW::jointStateCallback,
      this);

  imuSub_ = root_nh.subscribe(
      "/isaac/imu",
      1,
      &IsaacHW::imuCallback,
      this);

  // Backend odom from Python bridge.
  // Python publishes /isaac/odom only.
  odomSub_ = root_nh.subscribe(
      "/isaac/odom",
      1,
      &IsaacHW::odomCallback,
      this);

  contactSub_ = root_nh.subscribe(
      "/isaac/contacts",
      1,
      &IsaacHW::contactCallback,
      this);

  jointCmdPub_ = root_nh.advertise<std_msgs::Float64MultiArray>(
      "/isaac/joint_cmd",
      10);

  // Public Gazebo-style odom topics are published by IsaacHW only.
  groundTruthPub_ = root_nh.advertise<nav_msgs::Odometry>(
      "/ground_truth/state",
      10);

  // Do NOT advertise /odom here.
  // In this Isaac backend we publish only /ground_truth/state.
  // Publishing /odom from here caused mixed frame_id observations in /odom
  // when the base legged stack also handled odom internally.

  // Internal initialization only.
  // This does not command Isaac to stand.
  for (int i = 0; i < NUM_JOINTS; ++i) {
    jointData_[i].pos_ = GAZEBO_STAND_Q[i];
    jointData_[i].vel_ = 0.0;
    jointData_[i].tau_ = 0.0;

    jointData_[i].posDes_ = jointData_[i].pos_;
    jointData_[i].velDes_ = 0.0;
    jointData_[i].kp_ = 0.0;
    jointData_[i].kd_ = 0.0;
    jointData_[i].ff_ = 0.0;
  }

  for (int i = 0; i < NUM_LEGS; ++i) {
    contactState_[i] = true;
  }

  ROS_INFO("================================================");
  ROS_INFO("IsaacHW FIXED VERSION initialized.");
  ROS_INFO("C++ publishes q_des/dq_des/kp/kd/ff only.");
  ROS_INFO("Python publishes backend /isaac/odom.");
  ROS_INFO("C++ republishes public /ground_truth/state as world->base.");
  ROS_INFO("C++ does NOT publish public /odom.");
  ROS_INFO("C++ does NOT publish odom->base TF.");
  ROS_INFO("No automatic stand fallback when controller is inactive.");
  ROS_INFO("No Isaac stand convention correction in C++.");
  ROS_INFO("No forced minimum high gains in C++.");
  ROS_INFO("Q_DES_ALPHA=%.2f, FF_ALPHA=%.2f, MAX_FF_STEP=%.2f",
           Q_DES_ALPHA, FF_ALPHA, MAX_FF_STEP);
  ROS_INFO("Gains: MIN_KP=%.2f MIN_KD=%.2f MAX_KP=%.2f MAX_KD=%.2f MAX_FF=%.2f",
           MIN_ISAAC_KP,
           MIN_ISAAC_KD,
           MAX_ISAAC_KP,
           MAX_ISAAC_KD,
           MAX_ISAAC_FF);
  ROS_INFO("Expected order: LF_HAA LF_HFE LF_KFE LH_HAA LH_HFE LH_KFE RF_HAA RF_HFE RF_KFE RH_HAA RH_HFE RH_KFE");
  ROS_INFO("Subscribed: /isaac/joint_states, /isaac/imu, /isaac/odom, /isaac/contacts");
  ROS_INFO("Publishing: /isaac/joint_cmd, /ground_truth/state only from IsaacHW backend");
  ROS_INFO("USE_SAFE_STAND_FALLBACK_WHEN_NO_COMMAND = %s",
           USE_SAFE_STAND_FALLBACK_WHEN_NO_COMMAND ? "true" : "false");
  ROS_INFO("ENABLE_ISAAC_STAND_CONVENTION_CORRECTION = %s",
           ENABLE_ISAAC_STAND_CONVENTION_CORRECTION ? "true" : "false");
  ROS_INFO("================================================");

  return true;
}


void IsaacHW::read(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  if (jointStateReceived_) {
    std::array<bool, NUM_JOINTS> seen{};
    seen.fill(false);

    for (size_t j = 0; j < latestJointState_.name.size(); ++j) {
      const int index = isaacJointNameToIndex(latestJointState_.name[j]);

      if (index < 0 || index >= NUM_JOINTS) {
        ROS_WARN_THROTTLE(
            2.0,
            "Unknown Isaac joint name received: %s",
            latestJointState_.name[j].c_str());
        continue;
      }

      seen[index] = true;

      if (j < latestJointState_.position.size()) {
        jointData_[index].pos_ =
            safeValue(latestJointState_.position[j], jointData_[index].pos_);
      }

      if (j < latestJointState_.velocity.size()) {
        jointData_[index].vel_ =
            safeValue(latestJointState_.velocity[j], 0.0);
      }

      if (j < latestJointState_.effort.size()) {
        jointData_[index].tau_ =
            safeValue(latestJointState_.effort[j], 0.0);
      }
    }

    ROS_INFO_THROTTLE(
        1.0,
        "Isaac q: LF[%.3f %.3f %.3f] LH[%.3f %.3f %.3f] RF[%.3f %.3f %.3f] RH[%.3f %.3f %.3f]",
        jointData_[0].pos_, jointData_[1].pos_, jointData_[2].pos_,
        jointData_[3].pos_, jointData_[4].pos_, jointData_[5].pos_,
        jointData_[6].pos_, jointData_[7].pos_, jointData_[8].pos_,
        jointData_[9].pos_, jointData_[10].pos_, jointData_[11].pos_);

    ROS_INFO_THROTTLE(
        2.0,
        "Seen joints: LF[%d %d %d] LH[%d %d %d] RF[%d %d %d] RH[%d %d %d]",
        seen[0], seen[1], seen[2],
        seen[3], seen[4], seen[5],
        seen[6], seen[7], seen[8],
        seen[9], seen[10], seen[11]);

  } else {
    ROS_WARN_THROTTLE(
        1.0,
        "No /isaac/joint_states received yet. Internal state uses Gazebo stand q only as estimate.");
  }

  if (imuReceived_) {
    imuData_.ori_[0] = safeValue(latestImu_.orientation.x, 0.0);
    imuData_.ori_[1] = safeValue(latestImu_.orientation.y, 0.0);
    imuData_.ori_[2] = safeValue(latestImu_.orientation.z, 0.0);
    imuData_.ori_[3] = safeValue(latestImu_.orientation.w, 1.0);

    imuData_.angularVel_[0] = safeValue(latestImu_.angular_velocity.x, 0.0);
    imuData_.angularVel_[1] = safeValue(latestImu_.angular_velocity.y, 0.0);
    imuData_.angularVel_[2] = safeValue(latestImu_.angular_velocity.z, 0.0);

    imuData_.linearAcc_[0] = safeValue(latestImu_.linear_acceleration.x, 0.0);
    imuData_.linearAcc_[1] = safeValue(latestImu_.linear_acceleration.y, 0.0);
    imuData_.linearAcc_[2] = safeValue(latestImu_.linear_acceleration.z, 9.81);
  } else {
    ROS_WARN_THROTTLE(1.0, "No /isaac/imu received yet.");
  }

  if (odomReceived_) {
    // /isaac/odom comes from Python backend.
    // Publish only /ground_truth/state from this hardware backend.
    // /odom and odom->base TF are intentionally disabled here to avoid
    // mixed frame_id observations and duplicate odom paths.

    nav_msgs::Odometry gt = latestOdom_;
    gt.header.frame_id = "world";
    gt.child_frame_id = "base";

    // Publish only Gazebo-style ground truth.
    // Do NOT publish /odom here and do NOT broadcast odom->base TF here.
    // This avoids mixed /odom frames from two odom paths inside the same /isaac_hw process.
    groundTruthPub_.publish(gt);

    ROS_INFO_THROTTLE(
        1.0,
        "IsaacHW published /ground_truth/state world->base only | pos[%.3f %.3f %.3f] lin[%.4f %.4f %.4f] ang[%.4f %.4f %.4f]",
        gt.pose.pose.position.x,
        gt.pose.pose.position.y,
        gt.pose.pose.position.z,
        gt.twist.twist.linear.x,
        gt.twist.twist.linear.y,
        gt.twist.twist.linear.z,
        gt.twist.twist.angular.x,
        gt.twist.twist.angular.y,
        gt.twist.twist.angular.z);
  } else {
    ROS_WARN_THROTTLE(2.0, "No /isaac/odom received yet.");
  }

  if (!contactReceived_) {
    ROS_WARN_THROTTLE(
        2.0,
        "No /isaac/contacts received yet. Using default contact state.");
  }

  // Gazebo LeggedHWSim-like reset.
  // Controller update should overwrite these values if active.
  // If controller does not overwrite, write() sends zero gains/zero torque.
  for (int i = 0; i < NUM_JOINTS; ++i) {
    jointData_[i].posDes_ = jointData_[i].pos_;
    jointData_[i].velDes_ = 0.0;
    jointData_[i].kp_ = 0.0;
    jointData_[i].kd_ = 0.0;
    jointData_[i].ff_ = 0.0;
  }
}


void IsaacHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  std_msgs::Float64MultiArray cmdMsg;
  cmdMsg.data.resize(NUM_JOINTS * 5);

  std::array<double, NUM_JOINTS> qNow{};
  std::array<double, NUM_JOINTS> qDesRaw{};
  std::array<double, NUM_JOINTS> velDesRaw{};
  std::array<double, NUM_JOINTS> kpRaw{};
  std::array<double, NUM_JOINTS> kdRaw{};
  std::array<double, NUM_JOINTS> ffRaw{};

  for (int i = 0; i < NUM_JOINTS; ++i) {
    qNow[i] = safeValue(jointData_[i].pos_, GAZEBO_STAND_Q[i]);
    qDesRaw[i] = safeValue(jointData_[i].posDes_, qNow[i]);
    velDesRaw[i] = safeValue(jointData_[i].velDes_, 0.0);
    kpRaw[i] = safeValue(jointData_[i].kp_, 0.0);
    kdRaw[i] = safeValue(jointData_[i].kd_, 0.0);
    ffRaw[i] = safeValue(jointData_[i].ff_, 0.0);
  }

  const bool realCommand =
      hasRealControllerCommand(qDesRaw, qNow, kpRaw, kdRaw, ffRaw);

  std::array<double, NUM_JOINTS> qOut{};
  std::array<double, NUM_JOINTS> velOut{};
  std::array<double, NUM_JOINTS> kpOut{};
  std::array<double, NUM_JOINTS> kdOut{};
  std::array<double, NUM_JOINTS> ffOut{};

  if (!realCommand) {
    if (USE_SAFE_STAND_FALLBACK_WHEN_NO_COMMAND) {
      for (int i = 0; i < NUM_JOINTS; ++i) {
        qOut[i] = GAZEBO_STAND_Q[i];
        velOut[i] = 0.0;
        kpOut[i] = 20.0;
        kdOut[i] = 2.0;
        ffOut[i] = 0.0;
      }
    } else {
      // No command means no torque.
      for (int i = 0; i < NUM_JOINTS; ++i) {
        qOut[i] = qNow[i];
        velOut[i] = 0.0;
        kpOut[i] = 0.0;
        kdOut[i] = 0.0;
        ffOut[i] = 0.0;
      }
    }
  } else {
    for (int i = 0; i < NUM_JOINTS; ++i) {
      qOut[i] = qDesRaw[i];
      velOut[i] = velDesRaw[i];

      kpOut[i] = clampValue(kpRaw[i], 0.0, MAX_ISAAC_KP);
      kdOut[i] = clampValue(kdRaw[i], 0.0, MAX_ISAAC_KD);
      ffOut[i] = clampValue(ffRaw[i], -MAX_ISAAC_FF, MAX_ISAAC_FF);

      kpOut[i] = std::max(kpOut[i], MIN_ISAAC_KP);
      kdOut[i] = std::max(kdOut[i], MIN_ISAAC_KD);
    }
  }

  static bool smoothingInitialized = false;
  static std::array<double, NUM_JOINTS> qSmooth{};
  static std::array<double, NUM_JOINTS> ffSmooth{};

  if (!realCommand) {
    smoothingInitialized = false;
  } else {
    if (!smoothingInitialized) {
      qSmooth = qOut;
      ffSmooth = ffOut;
      smoothingInitialized = true;
    } else {
      for (int i = 0; i < NUM_JOINTS; ++i) {
        qSmooth[i] =
            Q_DES_ALPHA * qOut[i] +
            (1.0 - Q_DES_ALPHA) * qSmooth[i];

        const double ffFiltered =
            FF_ALPHA * ffOut[i] +
            (1.0 - FF_ALPHA) * ffSmooth[i];

        ffSmooth[i] =
            rateLimit(ffFiltered, ffSmooth[i], MAX_FF_STEP);
      }

      qOut = qSmooth;
      ffOut = ffSmooth;
    }
  }

  for (int i = 0; i < NUM_JOINTS; ++i) {
    cmdMsg.data[5 * i + 0] = qOut[i];
    cmdMsg.data[5 * i + 1] = velOut[i];
    cmdMsg.data[5 * i + 2] = kpOut[i];
    cmdMsg.data[5 * i + 3] = kdOut[i];
    cmdMsg.data[5 * i + 4] = ffOut[i];
  }

  jointCmdPub_.publish(cmdMsg);

  ROS_INFO_THROTTLE(
      1.0,
      "Isaac command mode: %s",
      realCommand ? "CONTROLLER ACTIVE - COMMAND PACKAGED" : "NO COMMAND - ZERO TORQUE");

  ROS_INFO_THROTTLE(
      1.0,
      "Isaac raw q_des: LF[%.3f %.3f %.3f] LH[%.3f %.3f %.3f] RF[%.3f %.3f %.3f] RH[%.3f %.3f %.3f]",
      qDesRaw[0], qDesRaw[1], qDesRaw[2],
      qDesRaw[3], qDesRaw[4], qDesRaw[5],
      qDesRaw[6], qDesRaw[7], qDesRaw[8],
      qDesRaw[9], qDesRaw[10], qDesRaw[11]);

  ROS_INFO_THROTTLE(
      1.0,
      "Isaac sent q_des: LF[%.3f %.3f %.3f] LH[%.3f %.3f %.3f] RF[%.3f %.3f %.3f] RH[%.3f %.3f %.3f]",
      cmdMsg.data[0], cmdMsg.data[5], cmdMsg.data[10],
      cmdMsg.data[15], cmdMsg.data[20], cmdMsg.data[25],
      cmdMsg.data[30], cmdMsg.data[35], cmdMsg.data[40],
      cmdMsg.data[45], cmdMsg.data[50], cmdMsg.data[55]);

  ROS_INFO_THROTTLE(
      1.0,
      "Isaac sent kp/kd/ff: kp LF[%.2f %.2f %.2f] LH[%.2f %.2f %.2f] RF[%.2f %.2f %.2f] RH[%.2f %.2f %.2f] | kd LF[%.2f %.2f %.2f] LH[%.2f %.2f %.2f] RF[%.2f %.2f %.2f] RH[%.2f %.2f %.2f] | ff LF[%.2f %.2f %.2f] LH[%.2f %.2f %.2f] RF[%.2f %.2f %.2f] RH[%.2f %.2f %.2f]",
      cmdMsg.data[2], cmdMsg.data[7], cmdMsg.data[12],
      cmdMsg.data[17], cmdMsg.data[22], cmdMsg.data[27],
      cmdMsg.data[32], cmdMsg.data[37], cmdMsg.data[42],
      cmdMsg.data[47], cmdMsg.data[52], cmdMsg.data[57],

      cmdMsg.data[3], cmdMsg.data[8], cmdMsg.data[13],
      cmdMsg.data[18], cmdMsg.data[23], cmdMsg.data[28],
      cmdMsg.data[33], cmdMsg.data[38], cmdMsg.data[43],
      cmdMsg.data[48], cmdMsg.data[53], cmdMsg.data[58],

      cmdMsg.data[4], cmdMsg.data[9], cmdMsg.data[14],
      cmdMsg.data[19], cmdMsg.data[24], cmdMsg.data[29],
      cmdMsg.data[34], cmdMsg.data[39], cmdMsg.data[44],
      cmdMsg.data[49], cmdMsg.data[54], cmdMsg.data[59]);
}


bool IsaacHW::setupJoints() {
  size_t registeredJoints = 0;

  for (const auto& joint : urdfModel_->joints_) {
    const int index = urdfJointNameToIndex(joint.first);

    if (index < 0 || index >= NUM_JOINTS) {
      continue;
    }

    hardware_interface::JointStateHandle stateHandle(
        joint.first,
        &jointData_[index].pos_,
        &jointData_[index].vel_,
        &jointData_[index].tau_);

    jointStateInterface_.registerHandle(stateHandle);

    hybridJointInterface_.registerHandle(
        HybridJointHandle(
            stateHandle,
            &jointData_[index].posDes_,
            &jointData_[index].velDes_,
            &jointData_[index].kp_,
            &jointData_[index].kd_,
            &jointData_[index].ff_));

    ROS_INFO(
        "Registered Isaac joint: %s -> controller index %d",
        joint.first.c_str(),
        index);

    registeredJoints++;
  }

  ROS_INFO("Total registered IsaacHW joints: %zu", registeredJoints);

  if (registeredJoints != NUM_JOINTS) {
    ROS_WARN(
        "Expected 12 registered joints, got %zu. Check URDF joint names.",
        registeredJoints);
  }

  return true;
}


bool IsaacHW::setupImu() {
  imuSensorInterface_.registerHandle(
      hardware_interface::ImuSensorHandle(
          "base_imu",
          "base_imu",
          imuData_.ori_,
          imuData_.oriCov_,
          imuData_.angularVel_,
          imuData_.angularVelCov_,
          imuData_.linearAcc_,
          imuData_.linearAccCov_));

  imuData_.ori_[0] = 0.0;
  imuData_.ori_[1] = 0.0;
  imuData_.ori_[2] = 0.0;
  imuData_.ori_[3] = 1.0;

  imuData_.angularVel_[0] = 0.0;
  imuData_.angularVel_[1] = 0.0;
  imuData_.angularVel_[2] = 0.0;

  imuData_.linearAcc_[0] = 0.0;
  imuData_.linearAcc_[1] = 0.0;
  imuData_.linearAcc_[2] = 9.81;

  for (int i = 0; i < 9; ++i) {
    imuData_.oriCov_[i] = 0.0;
    imuData_.angularVelCov_[i] = 0.0;
    imuData_.linearAccCov_[i] = 0.0;
  }

  imuData_.oriCov_[0] = 0.0012;
  imuData_.oriCov_[4] = 0.0012;
  imuData_.oriCov_[8] = 0.0012;

  imuData_.angularVelCov_[0] = 0.0004;
  imuData_.angularVelCov_[4] = 0.0004;
  imuData_.angularVelCov_[8] = 0.0004;

  imuData_.linearAccCov_[0] = 0.01;
  imuData_.linearAccCov_[4] = 0.01;
  imuData_.linearAccCov_[8] = 0.01;

  return true;
}


bool IsaacHW::setupContactSensor(ros::NodeHandle& /*nh*/) {
  for (size_t i = 0; i < ISAAC_CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(
        ContactSensorHandle(
            ISAAC_CONTACT_SENSOR_NAMES[i],
            &contactState_[i]));
  }

  for (int i = 0; i < NUM_LEGS; ++i) {
    contactState_[i] = true;
  }

  return true;
}


void IsaacHW::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  latestJointState_ = *msg;
  jointStateReceived_ = true;

  ROS_INFO_THROTTLE(
      2.0,
      "Received /isaac/joint_states with %zu joints.",
      latestJointState_.name.size());

  if (!latestJointState_.name.empty()) {
    std::string names;

    for (const auto& n : latestJointState_.name) {
      names += n + " ";
    }

    ROS_INFO_THROTTLE(
        5.0,
        "Isaac joint names: %s",
        names.c_str());
  }
}


void IsaacHW::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  latestImu_ = *msg;
  imuReceived_ = true;
}


void IsaacHW::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  latestOdom_ = *msg;
  odomReceived_ = true;
}


void IsaacHW::contactCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() < NUM_LEGS) {
    ROS_WARN_THROTTLE(
        1.0,
        "Received /isaac/contacts with size %zu, expected at least 4.",
        msg->data.size());
    return;
  }

  for (int i = 0; i < NUM_LEGS; ++i) {
    contactState_[i] = msg->data[i] > 0.5;
  }

  contactReceived_ = true;

  ROS_INFO_THROTTLE(
      1.0,
      "Isaac contacts: LF=%d LH=%d RF=%d RH=%d",
      contactState_[0],
      contactState_[1],
      contactState_[2],
      contactState_[3]);
}

}  // namespace legged
