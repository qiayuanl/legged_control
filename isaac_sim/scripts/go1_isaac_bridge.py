#!/usr/bin/env python3
"""
GO1 Isaac-ROS2 Bridge v9
=====================
Purpose of this version:
  - Keep Isaac/Gazebo-style ground-truth odom path clean.
  - Keep quaternion-derived yaw-rate for odom angular.z.
  - Use stable Isaac torque/FF smoothing tuned after the contact-phase and
    command-filter fixes.
  - Do NOT modify /cmd_vel or target generation here; use the separate
    isaac_gazebo_like_cmd_filter.py in ROS1 Docker for Gazebo-like path behavior.

Fixes applied vs previous versions:
  1. Robot trunk is reset to ZERO YAW and origin XY at initialization.
     The MPC initializes its reference from the first observation. Any
     nonzero yaw at startup becomes a permanent heading offset causing
     the robot to walk obliquely in the world frame.
  2. Odom twist.linear and twist.angular published in WORLD frame,
     matching legged_control rbd_state_ convention (linear_vel_x/y/z_global,
     angular_vel_x/y/z_global).
  3. IMU angular_velocity published in BODY frame (physical gyro convention).
  4. Quaternion finite-difference NOT used as angular velocity fallback.
     Roll/pitch oscillations during trot alias into fake yaw rate in the
     world frame and corrupt the Kalman filter with gait-locked yaw drift.
     Zero is published instead when the direct API is unavailable.
  5. IMU linear acceleration smoothed (ACC_ALPHA) to suppress foot-strike
     velocity spikes. Odom velocity is NOT smoothed.
  6. Direct API velocity fallback threshold changed from > 1e-10 to
     np.isfinite, so near-zero valid velocities during stance are not
     incorrectly replaced by finite-difference.
  7. TROT_PHASE_OFFSET = 0.0 (removed empirical 0.3 s offset that was
     run-specific and could swap the diagonal contact pair incorrectly).
"""

import builtins
import time
import math
import numpy as np
import omni
import omni.usd
import omni.kit.app
import rclpy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from pxr import UsdGeom, UsdPhysics, Sdf

try:
    import omni.physx
except Exception:
    omni.physx = None

try:
    import carb
except Exception:
    carb = None

try:
    from pxr import PhysxSchema
except Exception:
    PhysxSchema = None

try:
    from pxr import Gf
except Exception:
    Gf = None

try:
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction
except Exception:
    from omni.isaac.core.articulations import Articulation as SingleArticulation
    from omni.isaac.core.utils.types import ArticulationAction


# ==================================================
# CONFIG
# ==================================================

ARTICULATION_ROOT = "/World/go1/trunk"
BASE_PRIM_PATH    = "/World/go1/trunk"

# The go1 top-level prim (parent of trunk). The yaw reset is applied here
# so the entire robot moves, not just the trunk link in isolation.
ROBOT_ROOT_PRIM_PATH = "/World/go1"

CMD_TOPIC        = "/isaac/joint_cmd"
IMU_TOPIC        = "/isaac/imu"
CONTACT_TOPIC    = "/isaac/contacts_debug"   # debug only — real contacts from mode_to_contacts.py
ODOM_TOPIC       = "/isaac/odom"
JOINT_STATE_TOPIC = "/isaac/joint_states"

PHYSICS_HZ = 250.0
PHYSICS_DT = 1.0 / PHYSICS_HZ

FOOT_PRIMS = {
    "LF": "/World/go1/FL_foot",
    "LH": "/World/go1/RL_foot",
    "RF": "/World/go1/FR_foot",
    "RH": "/World/go1/RR_foot",
}

GROUND_Z      = 0.0
CONTACT_ON_Z  = 0.030
CONTACT_OFF_Z = 0.060

# ==================================================
# SPAWN POSE
# ==================================================
# Reset the robot to this position and zero yaw at bridge init.
# Adjust XY if your ground plane is not at the origin.
# Z is the nominal trunk height above ground for the GO1 standing pose.
SPAWN_X = 0.0
SPAWN_Y = 0.0
SPAWN_Z = 0.35   # metres above ground — adjust to your USD ground plane

# ==================================================
# SCHEDULED TROT CONTACTS
# ==================================================
# Published to CONTACT_TOPIC (/isaac/contacts_debug) only.
# Real /isaac/contacts must come from ROS1 mode_to_contacts.py.
USE_SCHEDULED_TROT_CONTACTS = True
TROT_PERIOD      = 0.6
TROT_SWITCH_TIME = 0.3
TROT_PHASE_OFFSET = 0.0   # no empirical offset — anchored to sim time

# ==================================================
# JOINT ORDER MAPPING
# ==================================================
# OCS2/ROS order → Isaac USD joint name
ROS_ORDER_TO_ISAAC_NAME = {
    0:  "FL_hip_joint",
    1:  "FL_thigh_joint",
    2:  "FL_calf_joint",
    3:  "RL_hip_joint",
    4:  "RL_thigh_joint",
    5:  "RL_calf_joint",
    6:  "FR_hip_joint",
    7:  "FR_thigh_joint",
    8:  "FR_calf_joint",
    9:  "RR_hip_joint",
    10: "RR_thigh_joint",
    11: "RR_calf_joint",
}

# Initial joint positions in ROS/OCS2 order (set once at init, not enforced).
GAZEBO_REST_Q_ROS_ORDER = np.array([
     0.33545,  1.19854, -2.81800,   # LF
     0.33541,  1.19855, -2.81800,   # LH
    -0.33546,  1.19852, -2.81800,   # RF
    -0.33541,  1.19855, -2.81800,   # RH
], dtype=np.float64)

SET_REST_POSE_ON_INIT = True

# ==================================================
# COMMAND / ACTUATION LIMITS
# ==================================================
MAX_ABS_KP = 60.0
MAX_ABS_KD = 8.0
MAX_ABS_FF = 35.0
MIN_ACTIVE_KP = 0.0
MIN_ACTIVE_KD = 0.0

# ==================================================
# TORQUE CALCULATION
# ==================================================
# tau = kp*(q_des - q) + kd*(dq_des - dq) + ff
# USD DriveAPI PD is disabled; Isaac receives final torque via ArticulationAction.
MAX_JOINT_TORQUE = 22.0

# Stable Isaac torque smoothing.
# Old 0.45/3.0 was too laggy and accumulated yaw.
# Aggressive 1.0/100.0 vibrated in stance/trot.
# This setting is the current stable compromise.
TAU_ALPHA        = 0.55
MAX_TAU_STEP     = 2.5

# ==================================================
# REAR SUPPORT BOOST (disabled by default)
# ==================================================
REAR_SUPPORT_GAIN_BOOST = False
REAR_SUPPORT_JOINTS     = [4, 5, 10, 11]
REAR_HFE_KP_BOOST       = 1.25
REAR_KFE_KP_BOOST       = 1.35
REAR_HFE_KD_BOOST       = 1.15
REAR_KFE_KD_BOOST       = 1.20
REAR_DRIVE_FORCE_BOOST  = 1.30

# ==================================================
# COMMAND SMOOTHING
# ==================================================
Q_DES_ALPHA  = 1.0    # 1.0 = pass-through (no position smoothing)
DQ_DES_ALPHA = 1.0
FF_ALPHA     = 0.30

MAX_Q_STEP   = 0.15
MAX_DQ_STEP  = 4.0
MAX_FF_STEP  = 1.5

COMMAND_TIMEOUT_SEC = 0.60

# ==================================================
# PUBLISH RATES
# ==================================================
JOINT_STATE_EVERY_N = 1
IMU_ODOM_EVERY_N    = 1
CONTACT_EVERY_N     = 1

# IMU accelerometer smoothing — applied to finite-diff acc only.
# Odom velocity is NOT smoothed.
ACC_ALPHA = 0.30

# Yaw drift test fix:
# If Isaac reports tiny angular velocity while the robot is physically still,
# zero it before publishing to prevent gyro integration drift.
OMEGA_DEADBAND = 0.01  # rad/s

# Most impactful yaw fix:
# Use quaternion-derived yaw-rate for omega_world.z, then low-pass it.
YAW_RATE_ALPHA = 0.15

# ==================================================
# DRIVE FORCE LIMITS (kept for reference, drives are disabled)
# ==================================================
MIN_DRIVE_FORCE   = 110.0
EXTRA_DRIVE_FORCE = 55.0
MAX_DRIVE_FORCE   = 170.0


# ==================================================
# CLEANUP OLD BRIDGE INSTANCES
# ==================================================

BRIDGE_NAME = "GO1_ISAAC_BRIDGE_V9_GAZEBO_STYLE_STABLE_TORQUE"

OLD_BRIDGE_NAMES = [
    "GO1_ISAAC_BRIDGE_STABLE_HYBRID_USD_PD_FF",
    "GO1_ISAAC_BRIDGE_STABLE_HYBRID_TROT_REAR_SUPPORT",
    "GO1_ISAAC_BRIDGE_PHYSX_HYBRID_USD_PD_FF",
    "GO1_ISAAC_BRIDGE_FORCE_PHYSX_250_HYBRID",
    "GO1_ISAAC_BRIDGE_FORCE_PHYSX_250_HYBRID_FIXED",
    "GO1_ISAAC_BRIDGE_FORCE_PHYSX_250_HYBRID_ODOM_BODY_TWIST",
    "GO1_ISAAC_BRIDGE_FORCE_PHYSX_250_HYBRID_GAZEBO_MATCH",
    "GO1_ISAAC_BRIDGE_FORCE_PHYSX_250_HYBRID_LEGGED_GLOBAL_ODOM",
    "GO1_ISAAC_BRIDGE_V5_ZERO_YAW_SPAWN",
    "GO1_ISAAC_BRIDGE_V6_OMEGA_DEADBAND_COV",
    "GO1_ISAAC_BRIDGE_V8_GAZEBO_STYLE_TORQUE_TEST",
    "GO1_ISAAC_BRIDGE_V9_GAZEBO_STYLE_STABLE_TORQUE",
]

for _name in OLD_BRIDGE_NAMES:
    if hasattr(builtins, _name):
        _old = getattr(builtins, _name)
        for _key in ["update_sub", "physics_sub"]:
            try:
                _sub = _old.get(_key)
                if _sub is not None:
                    if hasattr(_sub, "unsubscribe"):
                        _sub.unsubscribe()
                    elif hasattr(_sub, "release"):
                        _sub.release()
            except Exception:
                pass
        try:
            if _old.get("node") is not None:
                _old["node"].destroy_node()
        except Exception:
            pass
        try:
            delattr(builtins, _name)
        except Exception:
            pass
        print(f"[GO1 BRIDGE] Removed old bridge: {_name}")


setattr(builtins, BRIDGE_NAME, {
    "node": None,
    "cmd_sub": None,
    "imu_pub": None,
    "contact_pub": None,
    "odom_pub": None,
    "joint_state_pub": None,
    "update_sub": None,
    "physics_sub": None,

    "latest_cmd": None,
    "command_received": False,
    "last_cmd_wall_time": None,

    "robot": None,
    "initialized": False,
    "joint_indices": None,
    "dof_names": None,
    "joint_prims": None,

    "last_time": None,
    "last_pos": None,
    "last_vel": None,
    "last_quat": None,

    # State for quaternion-derived yaw-rate fix
    "yaw_prev": None,
    "yaw_rate_smooth": 0.0,

    "acc_world_smooth": np.zeros(3, dtype=np.float64),
    "acc_initialized": False,

    "contact_state": np.ones(4, dtype=np.float64),

    "smooth_initialized": False,
    "q_des_smooth": np.zeros(12, dtype=np.float64),
    "dq_des_smooth": np.zeros(12, dtype=np.float64),
    "ff_smooth": np.zeros(12, dtype=np.float64),

    "last_q":           np.zeros(12, dtype=np.float64),
    "last_dq":          np.zeros(12, dtype=np.float64),
    "last_ff_applied":  np.zeros(12, dtype=np.float64),
    "last_tau_applied": np.zeros(12, dtype=np.float64),
    "last_kp_used":     np.zeros(12, dtype=np.float64),
    "last_kd_used":     np.zeros(12, dtype=np.float64),
    "last_drive_force": np.zeros(12, dtype=np.float64),

    "tau_smooth":             np.zeros(12, dtype=np.float64),
    "tau_smooth_initialized": False,
    "drives_disabled":        False,

    "update_count":    0,
    "last_print_time": 0.0,
    "callback_mode":   "unknown",
    "physics_forced":  False,
})

bridge = getattr(builtins, BRIDGE_NAME)


# ==================================================
# PHYSICS RATE
# ==================================================

def find_or_create_physics_scene(stage):
    for p in ["/World/physicsScene", "/World/PhysicsScene",
              "/physicsScene", "/PhysicsScene"]:
        prim = stage.GetPrimAtPath(p)
        if prim.IsValid():
            return prim
    for prim in stage.Traverse():
        try:
            if prim.GetTypeName() == "PhysicsScene":
                return prim
        except Exception:
            pass
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
    print("[GO1 BRIDGE] Created PhysicsScene at /World/physicsScene")
    return scene.GetPrim()


def force_physics_rate():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("[GO1 BRIDGE] No USD stage; cannot set physics rate yet.")
        return False
    try:
        scene_prim = find_or_create_physics_scene(stage)
        scene = UsdPhysics.Scene(scene_prim)
        try:
            scene.CreateTimeStepsPerSecondAttr().Set(float(PHYSICS_HZ))
            print(f"[GO1 BRIDGE] UsdPhysics timeStepsPerSecond = {PHYSICS_HZ}")
        except Exception as e:
            print("[GO1 BRIDGE] Could not set timeStepsPerSecond:", e)
        try:
            scene.CreateGravityDirectionAttr().Set((0.0, 0.0, -1.0))
            scene.CreateGravityMagnitudeAttr().Set(9.81)
        except Exception:
            pass
        if PhysxSchema is not None:
            try:
                physx_api = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
                if hasattr(physx_api, "CreateTimeStepsPerSecondAttr"):
                    physx_api.CreateTimeStepsPerSecondAttr().Set(float(PHYSICS_HZ))
                if hasattr(physx_api, "CreateEnableCCDAttr"):
                    physx_api.CreateEnableCCDAttr().Set(True)
                print("[GO1 BRIDGE] PhysxSceneAPI settings applied.")
            except Exception as e:
                print("[GO1 BRIDGE] PhysxSceneAPI error:", e)
        if carb is not None:
            try:
                settings = carb.settings.get_settings()
                settings.set("/physics/timeStepsPerSecond", float(PHYSICS_HZ))
                settings.set("/physics/minFrameRate", float(PHYSICS_HZ))
                print(f"[GO1 BRIDGE] carb physics settings = {PHYSICS_HZ} Hz")
            except Exception as e:
                print("[GO1 BRIDGE] carb settings error:", e)
        bridge["physics_forced"] = True
        return True
    except Exception as e:
        print("[GO1 BRIDGE] force_physics_rate failed:", e)
        return False


# ==================================================
# ROS 2 SETUP
# ==================================================

if not rclpy.ok():
    rclpy.init(args=None)

node = rclpy.create_node("go1_isaac_bridge_v9_gazebo_style_stable_torque")

imu_pub        = node.create_publisher(Imu,              IMU_TOPIC,         10)
contact_pub    = node.create_publisher(Float64MultiArray, CONTACT_TOPIC,    10)
odom_pub       = node.create_publisher(Odometry,          ODOM_TOPIC,       10)
joint_state_pub = node.create_publisher(JointState,       JOINT_STATE_TOPIC, 10)

bridge["node"]            = node
bridge["imu_pub"]         = imu_pub
bridge["contact_pub"]     = contact_pub
bridge["odom_pub"]        = odom_pub
bridge["joint_state_pub"] = joint_state_pub


# ==================================================
# QUATERNION HELPERS
# ==================================================

def quat_normalize(q):
    q = np.array(q, dtype=np.float64)
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1., 0., 0., 0.])


def quat_conjugate(q):
    q = np.array(q, dtype=np.float64)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], dtype=np.float64)


def quat_rotate(q, v):
    q  = quat_normalize(q)
    vq = np.array([0., v[0], v[1], v[2]])
    return quat_multiply(quat_multiply(q, vq), quat_conjugate(q))[1:]


def world_to_body(q_world_from_body, v_world):
    return quat_rotate(quat_conjugate(quat_normalize(q_world_from_body)), v_world)


def yaw_from_quat_wxyz(q):
    """
    Extract world yaw from quaternion [w, x, y, z].
    """
    q = quat_normalize(q)
    w, x, y, z = q
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# ==================================================
# USD HELPERS
# ==================================================

def get_world_pose_from_cache(cache, stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim not found: {prim_path}")
    mat   = cache.GetLocalToWorldTransform(prim)
    trans = mat.ExtractTranslation()
    pos   = np.array([trans[0], trans[1], trans[2]], dtype=np.float64)
    try:
        q_gf = mat.ExtractRotationQuat()
    except Exception:
        q_gf = mat.ExtractRotation().GetQuat()
    w    = q_gf.GetReal()
    imag = q_gf.GetImaginary()
    return pos, quat_normalize([w, imag[0], imag[1], imag[2]])


def get_sim_time():
    try:
        return float(omni.timeline.get_timeline_interface().get_current_time())
    except Exception:
        return time.perf_counter()


def rate_limit_array(target, previous, max_step):
    target   = np.array(target,   dtype=np.float64)
    previous = np.array(previous, dtype=np.float64)
    return np.clip(target, previous - max_step, previous + max_step)


# ==================================================
# SPAWN POSE RESET  ← KEY FIX
# ==================================================

def reset_robot_spawn_pose(stage):
    """
    Move the entire GO1 robot to (SPAWN_X, SPAWN_Y, SPAWN_Z) with zero yaw.

    The MPC initializes its reference trajectory from the very first
    observation. If the robot spawns with any yaw offset (e.g. +9.2° as
    seen in the diagnostics), that offset becomes the MPC's idea of
    "forward" and the robot walks obliquely in the world frame for the
    entire run.

    We reset at the robot root prim (ROBOT_ROOT_PRIM_PATH = /World/go1)
    so that all child links move together. Resetting only /World/go1/trunk
    would leave the visual/collision meshes behind.
    """
    if Gf is None:
        print("[GO1 BRIDGE] pxr.Gf not available; cannot reset spawn pose.")
        return False

    # Try robot root first, fall back to trunk prim.
    for prim_path in [ROBOT_ROOT_PRIM_PATH, BASE_PRIM_PATH]:
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            try:
                xformable = UsdGeom.Xformable(prim)

                # Clear existing xform ops so we start clean.
                xformable.ClearXformOpOrder()

                # Translation — preserve XY spawn position, set Z to stand height.
                t_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
                t_op.Set(Gf.Vec3d(float(SPAWN_X), float(SPAWN_Y), float(SPAWN_Z)))

                # Rotation — identity quaternion = zero yaw/pitch/roll.
                r_op = xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
                r_op.Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))

                print(f"[GO1 BRIDGE] Spawn pose reset: prim={prim_path} "
                      f"pos=({SPAWN_X}, {SPAWN_Y}, {SPAWN_Z}) yaw=0")
                return True
            except Exception as e:
                print(f"[GO1 BRIDGE] Could not reset spawn pose at {prim_path}: {e}")

    print("[GO1 BRIDGE] WARNING: Could not find any prim to reset spawn pose.")
    return False


# ==================================================
# COMMAND CALLBACK
# ==================================================

def cmd_callback(msg):
    data = list(msg.data)
    if len(data) != 60:
        print(f"[GO1 BRIDGE] Ignored joint_cmd: expected 60 values, got {len(data)}")
        return

    arr  = np.array(data, dtype=np.float64).reshape((12, 5))
    q_des  = arr[:, 0].copy()
    dq_des = arr[:, 1].copy()
    kp     = np.clip(arr[:, 2], 0.0,        MAX_ABS_KP)
    kd     = np.clip(arr[:, 3], 0.0,        MAX_ABS_KD)
    ff     = np.clip(arr[:, 4], -MAX_ABS_FF, MAX_ABS_FF)

    active = bool(
        np.any(np.abs(kp) > 1e-6) or
        np.any(np.abs(kd) > 1e-6) or
        np.any(np.abs(ff) > 1e-6)
    )

    if active and REAR_SUPPORT_GAIN_BOOST:
        kp[4]  *= REAR_HFE_KP_BOOST;  kp[5]  *= REAR_KFE_KP_BOOST
        kp[10] *= REAR_HFE_KP_BOOST;  kp[11] *= REAR_KFE_KP_BOOST
        kd[4]  *= REAR_HFE_KD_BOOST;  kd[5]  *= REAR_KFE_KD_BOOST
        kd[10] *= REAR_HFE_KD_BOOST;  kd[11] *= REAR_KFE_KD_BOOST
        kp = np.clip(kp, 0.0, MAX_ABS_KP)
        kd = np.clip(kd, 0.0, MAX_ABS_KD)

    bridge["latest_cmd"] = {
        "q_des": q_des, "dq_des": dq_des,
        "kp": kp, "kd": kd, "ff": ff, "active": active,
    }
    bridge["command_received"]    = True
    bridge["last_cmd_wall_time"]  = time.perf_counter()


cmd_sub = node.create_subscription(Float64MultiArray, CMD_TOPIC, cmd_callback, 10)
bridge["cmd_sub"] = cmd_sub


# ==================================================
# INITIALIZE ARTICULATION
# ==================================================

def initialize_robot_if_needed():
    if bridge["initialized"]:
        return True

    stage     = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(ARTICULATION_ROOT)
    if not root_prim.IsValid():
        print(f"[GO1 BRIDGE] Articulation root not found: {ARTICULATION_ROOT}")
        return False

    try:
        force_physics_rate()

        # ── CRITICAL: reset spawn pose BEFORE initializing articulation ──
        # This ensures the first observation the MPC receives has zero yaw
        # so the controller's world frame matches Isaac's world frame.
        reset_robot_spawn_pose(stage)

        robot = SingleArticulation(
            prim_path=ARTICULATION_ROOT,
            name="go1_bridge_v9_robot",
        )
        robot.initialize()

        dof_names = list(robot.dof_names)
        print("[GO1 BRIDGE] Isaac DOF names:")
        for i, n in enumerate(dof_names):
            print(f"  {i}: {n}")

        joint_indices = []
        joint_prims   = []

        for ros_i in range(12):
            isaac_name = ROS_ORDER_TO_ISAAC_NAME[ros_i]
            if isaac_name not in dof_names:
                raise RuntimeError(f"Joint '{isaac_name}' not found in Isaac DOFs")
            joint_indices.append(dof_names.index(isaac_name))

            found_prim = None
            for prim in stage.Traverse():
                if prim.GetName() == isaac_name:
                    found_prim = prim
                    break
            if found_prim is None:
                raise RuntimeError(f"Could not find joint prim for '{isaac_name}'")
            joint_prims.append(found_prim)

        bridge["robot"]         = robot
        bridge["joint_indices"] = np.array(joint_indices, dtype=np.int32)
        bridge["dof_names"]     = dof_names
        bridge["joint_prims"]   = joint_prims

        if SET_REST_POSE_ON_INIT:
            q_all  = np.array(robot.get_joint_positions(),  dtype=np.float64)
            dq_all = np.array(robot.get_joint_velocities(), dtype=np.float64)
            for ros_i, isaac_i in enumerate(joint_indices):
                q_all[isaac_i]  = GAZEBO_REST_Q_ROS_ORDER[ros_i]
                dq_all[isaac_i] = 0.0
            robot.set_joint_positions(q_all)
            robot.set_joint_velocities(dq_all)
            try:
                robot.set_linear_velocity(np.zeros(3))
                robot.set_angular_velocity(np.zeros(3))
            except Exception:
                pass
            print("[GO1 BRIDGE] Initial rest pose set.")

        bridge["initialized"] = True
        print("[GO1 BRIDGE] Robot initialized. DOF index map:", joint_indices)
        print("[GO1 BRIDGE] Callback mode:", bridge["callback_mode"])
        print("[GO1 BRIDGE] Physics forced:", bridge["physics_forced"])
        return True

    except Exception as e:
        print("[GO1 BRIDGE] Initialization failed:", e)
        return False


# ==================================================
# READ JOINT STATE
# ==================================================

def read_joint_state_ros_order():
    robot         = bridge["robot"]
    joint_indices = bridge["joint_indices"]
    q_all  = np.array(robot.get_joint_positions(),  dtype=np.float64)
    dq_all = np.array(robot.get_joint_velocities(), dtype=np.float64)
    q  = np.zeros(12, dtype=np.float64)
    dq = np.zeros(12, dtype=np.float64)
    for ros_i in range(12):
        isaac_i    = joint_indices[ros_i]
        q[ros_i]   = q_all[isaac_i]
        dq[ros_i]  = dq_all[isaac_i]
    bridge["last_q"]  = q.copy()
    bridge["last_dq"] = dq.copy()
    return q, dq


# ==================================================
# COMMAND SMOOTHING
# ==================================================

def get_smoothed_command(cmd):
    q_des  = cmd["q_des"];  dq_des = cmd["dq_des"]
    kp     = cmd["kp"];     kd     = cmd["kd"];    ff = cmd["ff"]
    active = cmd["active"]

    if not active:
        bridge["smooth_initialized"] = False
        return q_des, dq_des, kp, kd, np.zeros(12, dtype=np.float64)

    if not bridge["smooth_initialized"]:
        bridge["q_des_smooth"]  = q_des.copy()
        bridge["dq_des_smooth"] = dq_des.copy()
        bridge["ff_smooth"]     = ff.copy()
        bridge["smooth_initialized"] = True
    else:
        q_t  = Q_DES_ALPHA  * q_des  + (1 - Q_DES_ALPHA)  * bridge["q_des_smooth"]
        dq_t = DQ_DES_ALPHA * dq_des + (1 - DQ_DES_ALPHA) * bridge["dq_des_smooth"]
        ff_t = FF_ALPHA     * ff     + (1 - FF_ALPHA)     * bridge["ff_smooth"]
        bridge["q_des_smooth"]  = rate_limit_array(q_t,  bridge["q_des_smooth"],  MAX_Q_STEP)
        bridge["dq_des_smooth"] = rate_limit_array(dq_t, bridge["dq_des_smooth"], MAX_DQ_STEP)
        bridge["ff_smooth"]     = rate_limit_array(ff_t, bridge["ff_smooth"],     MAX_FF_STEP)

    return (bridge["q_des_smooth"].copy(), bridge["dq_des_smooth"].copy(),
            kp.copy(), kd.copy(), bridge["ff_smooth"].copy())


# ==================================================
# DISABLE USD DRIVES
# ==================================================

def disable_usd_drives():
    """Zero out all USD DriveAPI gains so Isaac uses pure torque from ArticulationAction."""
    joint_prims = bridge["joint_prims"]
    if joint_prims is None:
        return
    for ros_i in range(12):
        prim      = joint_prims[ros_i]
        drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive_api.CreateStiffnessAttr(0.0)
        drive_api.CreateDampingAttr(0.0)
        drive_api.CreateMaxForceAttr(0.0)
        drive_api.CreateTargetPositionAttr(0.0)
        drive_api.CreateTargetVelocityAttr(0.0)
    bridge["last_drive_force"] = np.zeros(12, dtype=np.float64)
    bridge["drives_disabled"]  = True


# ==================================================
# TORQUE COMPUTATION
# ==================================================

def command_is_timed_out():
    last = bridge.get("last_cmd_wall_time")
    return last is None or (time.perf_counter() - last) > COMMAND_TIMEOUT_SEC


def compute_hybrid_torque(q_des, dq_des, kp, kd, ff, q_now, dq_now, active):
    if not active:
        bridge["tau_smooth_initialized"] = False
        return np.zeros(12, dtype=np.float64)

    tau = kp * (q_des - q_now) + kd * (dq_des - dq_now) + ff

    if REAR_SUPPORT_GAIN_BOOST:
        for idx in REAR_SUPPORT_JOINTS:
            tau[idx] *= 1.08

    tau = np.clip(tau, -MAX_JOINT_TORQUE, MAX_JOINT_TORQUE)

    if not bridge["tau_smooth_initialized"]:
        bridge["tau_smooth"]             = tau.copy()
        bridge["tau_smooth_initialized"] = True
    else:
        tau_f = TAU_ALPHA * tau + (1.0 - TAU_ALPHA) * bridge["tau_smooth"]
        tau_f = rate_limit_array(tau_f, bridge["tau_smooth"], MAX_TAU_STEP)
        bridge["tau_smooth"] = tau_f.copy()
        tau = tau_f

    return tau


def apply_latest_command():
    cmd = bridge["latest_cmd"]
    if cmd is None:
        return
    if not initialize_robot_if_needed():
        return

    try:
        q_now, dq_now = read_joint_state_ros_order()

        if not bridge.get("drives_disabled", False):
            disable_usd_drives()

        zero = np.zeros(12, dtype=np.float64)

        if command_is_timed_out():
            bridge["tau_smooth_initialized"] = False
            bridge["robot"].apply_action(ArticulationAction(
                joint_efforts=zero, joint_indices=bridge["joint_indices"]))
            bridge["last_ff_applied"]  = zero.copy()
            bridge["last_tau_applied"] = zero.copy()
            bridge["last_kp_used"]     = zero.copy()
            bridge["last_kd_used"]     = zero.copy()
            return

        q_des, dq_des, kp, kd, ff = get_smoothed_command(cmd)
        active = bool(cmd["active"])

        tau = compute_hybrid_torque(q_des, dq_des, kp, kd, ff, q_now, dq_now, active)

        bridge["robot"].apply_action(ArticulationAction(
            joint_efforts=tau, joint_indices=bridge["joint_indices"]))

        bridge["last_ff_applied"]  = ff.copy()
        bridge["last_tau_applied"] = tau.copy()
        bridge["last_kp_used"]     = kp.copy()
        bridge["last_kd_used"]     = kd.copy()

    except Exception as e:
        print("[GO1 BRIDGE] apply_action failed:", e)


# ==================================================
# PUBLISH JOINT STATES
# ==================================================

def publish_joint_states():
    if not bridge["initialized"]:
        return
    try:
        q, dq = read_joint_state_ros_order()
    except Exception as e:
        print("[GO1 BRIDGE] Cannot read joint states:", e)
        return
    msg          = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name     = [ROS_ORDER_TO_ISAAC_NAME[i] for i in range(12)]
    msg.position = [float(x) for x in q]
    msg.velocity = [float(x) for x in dq]
    msg.effort   = [float(x) for x in bridge["last_tau_applied"]]
    joint_state_pub.publish(msg)


# ==================================================
# BASE VELOCITY HELPERS
# ==================================================

def _as_vec3(value):
    try:
        arr = np.array(value, dtype=np.float64).reshape(-1)
        if arr.size >= 3 and np.all(np.isfinite(arr[:3])):
            return arr[:3].copy()
    except Exception:
        pass
    return None


def get_robot_base_velocities_world(robot):
    lin = None
    ang = None
    for m in ["get_linear_velocity", "get_world_linear_velocity", "get_default_linear_velocity"]:
        try:
            fn = getattr(robot, m, None)
            if callable(fn):
                lin = _as_vec3(fn())
                if lin is not None:
                    break
        except Exception:
            pass
    for m in ["get_angular_velocity", "get_world_angular_velocity", "get_default_angular_velocity"]:
        try:
            fn = getattr(robot, m, None)
            if callable(fn):
                ang = _as_vec3(fn())
                if ang is not None:
                    break
        except Exception:
            pass
    if lin is None or ang is None:
        for m in ["get_velocities", "get_world_velocities"]:
            try:
                fn = getattr(robot, m, None)
                if callable(fn):
                    v = np.array(fn(), dtype=np.float64).reshape(-1)
                    if v.size >= 6 and np.all(np.isfinite(v[:6])):
                        if lin is None: lin = v[:3].copy()
                        if ang is None: ang = v[3:6].copy()
                        break
            except Exception:
                pass
    return lin, ang


# ==================================================
# PUBLISH IMU + ODOM
# ==================================================

def publish_imu_and_odom(stage, cache, now):
    try:
        pos_world, q_world_from_body = get_world_pose_from_cache(
            cache, stage, BASE_PRIM_PATH)
    except Exception as e:
        print("[GO1 BRIDGE] Cannot read base pose:", e)
        return

    last_time = bridge["last_time"]

    if last_time is None:
        bridge["last_time"]  = now
        bridge["last_pos"]   = pos_world
        bridge["last_vel"]   = np.zeros(3, dtype=np.float64)
        bridge["last_quat"]  = q_world_from_body
        return

    dt = now - last_time
    if dt <= 1e-6 or dt > 1.0:
        dt = PHYSICS_DT * IMU_ODOM_EVERY_N

    # ── Linear velocity ──────────────────────────────────────────────────
    # Prefer direct Isaac API (world frame). Fall back to finite-diff only
    # if the API returns non-finite values. Near-zero is a valid velocity
    # during stance — do NOT threshold on magnitude.
    vel_world_fd = (pos_world - bridge["last_pos"]) / dt

    robot            = bridge.get("robot")
    vel_world_direct = None
    omega_world_direct = None
    if robot is not None:
        vel_world_direct, omega_world_direct = get_robot_base_velocities_world(robot)

    if vel_world_direct is not None and np.all(np.isfinite(vel_world_direct)):
        vel_world = vel_world_direct
    else:
        vel_world = vel_world_fd

    # ── Angular velocity ─────────────────────────────────────────────────
    # CRITICAL: Do NOT use quaternion finite-difference as fallback.
    # During trot, roll/pitch oscillations alias into a fake world-frame
    # yaw rate that creates gait-locked oscillatory yaw drift in the
    # Kalman filter. Publish zero instead if the direct API is unavailable.
    if omega_world_direct is not None and np.all(np.isfinite(omega_world_direct)):
        omega_world = omega_world_direct.copy()
    else:
        omega_world = np.zeros(3, dtype=np.float64)

    # FIX A: deadband tiny numerical angular velocity at rest.
    # This prevents small PhysX/API noise from being integrated as yaw.
    if np.linalg.norm(omega_world) < OMEGA_DEADBAND:
        omega_world = np.zeros(3, dtype=np.float64)

    # FIX D, most important: replace world yaw rate with quaternion-derived
    # yaw finite difference only on Z, then low-pass it. This avoids using
    # the articulation angular-velocity Z component that can contain
    # foot-strike vibration and solver noise.
    yaw_now = yaw_from_quat_wxyz(q_world_from_body)
    yaw_prev = bridge.get("yaw_prev", None)

    if yaw_prev is None:
        yaw_rate_raw = 0.0
        bridge["yaw_rate_smooth"] = 0.0
    else:
        dyaw = wrap_pi(yaw_now - yaw_prev)
        yaw_rate_raw = dyaw / dt
        if abs(yaw_rate_raw) < OMEGA_DEADBAND:
            yaw_rate_raw = 0.0
        bridge["yaw_rate_smooth"] = (
            YAW_RATE_ALPHA * yaw_rate_raw
            + (1.0 - YAW_RATE_ALPHA) * bridge.get("yaw_rate_smooth", 0.0)
        )

    bridge["yaw_prev"] = yaw_now

    # Keep roll/pitch angular velocity from direct API; use clean yaw-rate.
    omega_world[2] = bridge["yaw_rate_smooth"]

    # ── IMU acceleration ─────────────────────────────────────────────────
    # Finite-diff of velocity amplifies foot-strike spikes. Smooth the
    # acceleration for the IMU only; odom velocity stays unsmoothed.
    acc_raw = (vel_world - bridge["last_vel"]) / dt
    if not bridge["acc_initialized"]:
        bridge["acc_world_smooth"] = acc_raw.copy()
        bridge["acc_initialized"]  = True
    else:
        bridge["acc_world_smooth"] = (
            ACC_ALPHA * acc_raw + (1.0 - ACC_ALPHA) * bridge["acc_world_smooth"])
    acc_world = bridge["acc_world_smooth"]

    # Specific force and body-frame quantities for IMU message
    gravity_world        = np.array([0., 0., -9.81])
    specific_force_world = acc_world - gravity_world
    specific_force_body  = world_to_body(q_world_from_body, specific_force_world)
    omega_body           = world_to_body(q_world_from_body, omega_world)

    # Apply the same deadband after rotating to IMU/body frame.
    if np.linalg.norm(omega_body) < OMEGA_DEADBAND:
        omega_body = np.zeros(3, dtype=np.float64)

    stamp = node.get_clock().now().to_msg()

    # ── IMU message ───────────────────────────────────────────────────────
    # orientation : world frame (quaternion)
    # angular_velocity : BODY frame  (physical gyro convention)
    # linear_acceleration : BODY frame specific force (includes gravity removal)
    imu_msg = Imu()
    imu_msg.header.stamp    = stamp
    imu_msg.header.frame_id = "base_imu"

    imu_msg.orientation.w = float(q_world_from_body[0])
    imu_msg.orientation.x = float(q_world_from_body[1])
    imu_msg.orientation.y = float(q_world_from_body[2])
    imu_msg.orientation.z = float(q_world_from_body[3])

    imu_msg.angular_velocity.x = float(omega_body[0])   # body frame
    imu_msg.angular_velocity.y = float(omega_body[1])
    imu_msg.angular_velocity.z = float(omega_body[2])

    imu_msg.linear_acceleration.x = float(specific_force_body[0])  # body frame
    imu_msg.linear_acceleration.y = float(specific_force_body[1])
    imu_msg.linear_acceleration.z = float(specific_force_body[2])

    imu_msg.orientation_covariance[0]    = 0.0012
    imu_msg.orientation_covariance[4]    = 0.0012
    imu_msg.orientation_covariance[8]    = 0.0012
    # FIX B: do not over-trust simulated gyro yaw rate.
    imu_msg.angular_velocity_covariance[0]  = 0.002
    imu_msg.angular_velocity_covariance[4]  = 0.002
    imu_msg.angular_velocity_covariance[8]  = 0.002
    imu_msg.linear_acceleration_covariance[0] = 0.01
    imu_msg.linear_acceleration_covariance[4] = 0.01
    imu_msg.linear_acceleration_covariance[8] = 0.01

    imu_pub.publish(imu_msg)

    # ── Odometry message ──────────────────────────────────────────────────
    # legged_control rbd_state_ convention (confirmed from issue #10):
    #   linear_vel_x/y/z_global  → twist.linear  in WORLD frame
    #   angular_vel_x/y/z_global → twist.angular in WORLD frame
    odom_msg = Odometry()
    odom_msg.header.stamp    = stamp
    odom_msg.header.frame_id = "world"
    odom_msg.child_frame_id  = "base"

    odom_msg.pose.pose.position.x = float(pos_world[0])
    odom_msg.pose.pose.position.y = float(pos_world[1])
    odom_msg.pose.pose.position.z = float(pos_world[2])

    odom_msg.pose.pose.orientation.w = float(q_world_from_body[0])
    odom_msg.pose.pose.orientation.x = float(q_world_from_body[1])
    odom_msg.pose.pose.orientation.y = float(q_world_from_body[2])
    odom_msg.pose.pose.orientation.z = float(q_world_from_body[3])

    odom_msg.twist.twist.linear.x  = float(vel_world[0])   # world frame
    odom_msg.twist.twist.linear.y  = float(vel_world[1])
    odom_msg.twist.twist.linear.z  = float(vel_world[2])

    odom_msg.twist.twist.angular.x = float(omega_world[0])  # world frame
    odom_msg.twist.twist.angular.y = float(omega_world[1])
    odom_msg.twist.twist.angular.z = float(omega_world[2])

    odom_msg.pose.covariance[0]  = 0.00055
    odom_msg.pose.covariance[7]  = 0.00055
    odom_msg.pose.covariance[14] = 0.000053
    odom_msg.pose.covariance[21] = 0.0012
    odom_msg.pose.covariance[28] = 0.0012
    odom_msg.pose.covariance[35] = 0.0012

    odom_msg.twist.covariance[0]  = 0.00049
    odom_msg.twist.covariance[7]  = 0.00049
    odom_msg.twist.covariance[14] = 0.00049
    # FIX C: do not over-trust odom yaw-rate twist.
    odom_msg.twist.covariance[21] = 0.002
    odom_msg.twist.covariance[28] = 0.002
    odom_msg.twist.covariance[35] = 0.002

    odom_pub.publish(odom_msg)

    bridge["last_time"]  = now
    bridge["last_pos"]   = pos_world
    bridge["last_vel"]   = vel_world
    bridge["last_quat"]  = q_world_from_body


# ==================================================
# PUBLISH CONTACTS (debug only)
# ==================================================

def publish_contacts(stage, cache):
    if USE_SCHEDULED_TROT_CONTACTS:
        now   = get_sim_time()
        phase = (now + TROT_PHASE_OFFSET) % TROT_PERIOD
        if phase < TROT_SWITCH_TIME:
            contacts = np.array([1., 0., 0., 1.])  # mode 9: LF + RH
        else:
            contacts = np.array([0., 1., 1., 0.])  # mode 6: LH + RF
    else:
        contacts = bridge["contact_state"].copy()
        for idx, leg in enumerate(["LF", "LH", "RF", "RH"]):
            try:
                foot_pos, _ = get_world_pose_from_cache(cache, stage, FOOT_PRIMS[leg])
                foot_z = foot_pos[2]
                if   foot_z <= (GROUND_Z + CONTACT_ON_Z):  contacts[idx] = 1.0
                elif foot_z >= (GROUND_Z + CONTACT_OFF_Z): contacts[idx] = 0.0
            except Exception as e:
                print(f"[GO1 BRIDGE] Cannot read foot {leg}: {e}")

    bridge["contact_state"] = contacts
    msg      = Float64MultiArray()
    msg.data = [float(x) for x in contacts]
    contact_pub.publish(msg)


# ==================================================
# MAIN STEP
# ==================================================

def bridge_step(dt=None):
    stage = omni.usd.get_context().get_stage()
    cache = UsdGeom.XformCache()
    now   = get_sim_time()

    bridge["update_count"] += 1
    k = bridge["update_count"]

    try:
        rclpy.spin_once(node, timeout_sec=0.0)
    except Exception as e:
        print("[GO1 BRIDGE] rclpy spin error:", e)
        return

    if not initialize_robot_if_needed():
        return

    apply_latest_command()

    if k % JOINT_STATE_EVERY_N == 0:
        publish_joint_states()
    if k % IMU_ODOM_EVERY_N == 0:
        publish_imu_and_odom(stage, cache, now)
    if k % CONTACT_EVERY_N == 0:
        publish_contacts(stage, cache)

    if now - bridge["last_print_time"] > 1.5:
        bridge["last_print_time"] = now
        has_cmd   = bridge["latest_cmd"] is not None
        timed_out = command_is_timed_out()
        print("------------------------------------------------")
        print("[GO1 BRIDGE] running  |  updates:", bridge["update_count"])
        print("callback_mode:", bridge["callback_mode"],
              "| physics_forced:", bridge["physics_forced"])
        print("cmd:", has_cmd, "| timeout:", timed_out)
        if has_cmd:
            cmd = bridge["latest_cmd"]
            print("active:", cmd["active"])
            print("kp   :", np.round(bridge["last_kp_used"],  2))
            print("kd   :", np.round(bridge["last_kd_used"],  2))
            print("ff   :", np.round(bridge["last_ff_applied"], 2))
            print("tau  :", np.round(bridge["last_tau_applied"], 2))
            print("q_now:", np.round(bridge["last_q"],  3))
            print("q_des:", np.round(bridge["q_des_smooth"], 3))
        print("contacts:", bridge["contact_state"])
        print("------------------------------------------------")


# ==================================================
# REGISTER PHYSICS CALLBACK
# ==================================================

def register_callback():
    force_physics_rate()

    # 1. PhysX physics-step interface (preferred — runs at physics rate)
    try:
        if omni.physx is None:
            raise RuntimeError("omni.physx unavailable")
        physx_iface = omni.physx.get_physx_interface()
        if hasattr(physx_iface, "subscribe_physics_step_events"):
            sub = physx_iface.subscribe_physics_step_events(lambda dt: bridge_step(dt))
            bridge["physics_sub"] = sub
            bridge["update_sub"]  = sub
            bridge["callback_mode"] = "physx_interface.subscribe_physics_step_events"
            print("[GO1 BRIDGE] Registered PhysX physics-step callback.")
            return
        raise RuntimeError("physx_interface has no subscribe_physics_step_events")
    except Exception as e:
        print("[GO1 BRIDGE] PhysX interface callback failed:", e)

    # 2. PhysX simulation-step interface (fallback)
    try:
        if omni.physx is None:
            raise RuntimeError("omni.physx unavailable")
        sim_iface = omni.physx.get_physx_simulation_interface()
        if hasattr(sim_iface, "subscribe_physics_step_events"):
            sub = sim_iface.subscribe_physics_step_events(lambda dt: bridge_step(dt))
            bridge["physics_sub"] = sub
            bridge["update_sub"]  = sub
            bridge["callback_mode"] = "physx_simulation_interface.subscribe_physics_step_events"
            print("[GO1 BRIDGE] Registered PhysX simulation-step callback.")
            return
        raise RuntimeError("sim_interface has no subscribe_physics_step_events")
    except Exception as e:
        print("[GO1 BRIDGE] PhysX simulation callback failed:", e)

    # 3. App update loop (last resort — not physics-rate locked)
    sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        lambda event: bridge_step(None),
        name="go1_isaac_bridge_v9_update",
    )
    bridge["update_sub"]  = sub
    bridge["callback_mode"] = "app_update_fallback"
    print("[GO1 BRIDGE] WARNING: app-update fallback — feedback rates may be low.")


register_callback()


# ==================================================
# STARTUP SUMMARY
# ==================================================
print("=" * 60)
print("GO1 ISAAC BRIDGE v9  —  GAZEBO-STYLE STATE + STABLE TORQUE")
print("=" * 60)
print(f"Articulation root : {ARTICULATION_ROOT}")
print(f"Robot root prim   : {ROBOT_ROOT_PRIM_PATH}")
print(f"Spawn pose        : x={SPAWN_X} y={SPAWN_Y} z={SPAWN_Z} yaw=0")
print(f"Sub  : {CMD_TOPIC}")
print(f"Pubs : {JOINT_STATE_TOPIC}  {IMU_TOPIC}  {ODOM_TOPIC}  {CONTACT_TOPIC}")
print("NOTE : Real /isaac/contacts must come from ROS1 mode_to_contacts.py")
print("Odom twist frame  : WORLD (legged_control rbd_state_ convention)")
print("IMU  ang_vel frame: BODY  (physical gyro convention)")
print("IMU  lin_acc frame: BODY  (specific force, gravity removed)")
print(f"Physics Hz        : {PHYSICS_HZ}")
print(f"TAU_ALPHA         : {TAU_ALPHA}   MAX_TAU_STEP: {MAX_TAU_STEP}")
print(f"FF_ALPHA          : {FF_ALPHA}    ACC_ALPHA   : {ACC_ALPHA}")
print(f"OMEGA_DEADBAND    : {OMEGA_DEADBAND} rad/s")
print(f"YAW_RATE_ALPHA    : {YAW_RATE_ALPHA}")
print(f"MAX_JOINT_TORQUE  : {MAX_JOINT_TORQUE} Nm")
print(f"COMMAND_TIMEOUT   : {COMMAND_TIMEOUT_SEC} s")
print(f"Trot contacts     : period={TROT_PERIOD}s  switch={TROT_SWITCH_TIME}s  offset={TROT_PHASE_OFFSET}s")
print(f"Callback mode     : {bridge['callback_mode']}")
print(f"Physics forced    : {bridge['physics_forced']}")
print("=" * 60)
