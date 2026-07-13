#!/usr/bin/env python3

import math
import time
import rospy
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import mpc_observation


# ============================================================
# Isaac Gazebo-like command filter v6
# ============================================================
#
# IMPORTANT:
#   This version DOES NOT publish /cmd_vel by itself.
#
# It publishes only when /cmd_vel_user is fresh.
#
# Behavior:
#   - No /cmd_vel_user command:
#       publish nothing
#
#   - linear.z > STAND_Z_THRESHOLD:
#       stand / hold current x,y,yaw
#
#   - linear.x or linear.y:
#       walking with yaw + cross-track correction
#
#   - angular.z:
#       pass-through yaw rotation
#
# This prevents the robot from standing by itself.
# ============================================================


# Walking yaw correction
K_YAW = 1.20
MAX_WZ_CORR = 0.55

# Walking cross-track correction
USE_CROSSTRACK_CORRECTION = True
K_CROSSTRACK = 0.45
MAX_V_CORR = 0.15

# Explicit stand/hold correction
USE_STAND_HOLD = True
K_HOLD_POS = 0.65
MAX_HOLD_V = 0.12

K_HOLD_YAW = 1.20
MAX_HOLD_WZ = 0.45

# Thresholds
MIN_SPEED_TO_LOCK_PATH = 0.02
TURN_CMD_THRESHOLD = 0.03

# If command publisher stops, this filter stops publishing /cmd_vel.
COMMAND_TIMEOUT_SEC = 0.35

# linear.z > 0.5 means explicit stand/hold request
STAND_Z_THRESHOLD = 0.5

# Relock path if command direction changes
RELOCK_DIR_ANGLE_DEG = 20.0
RELOCK_DIR_DOT = math.cos(math.radians(RELOCK_DIR_ANGLE_DEG))


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def rot2d(yaw, vx, vy):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * vx - s * vy, s * vx + c * vy


def limit_vec2(x, y, max_norm):
    n = math.sqrt(x * x + y * y)
    if n > max_norm and n > 1e-9:
        scale = max_norm / n
        return x * scale, y * scale
    return x, y


class IsaacGazeboLikeCmdFilter:
    def __init__(self):
        rospy.init_node("isaac_gazebo_like_cmd_filter", anonymous=False)

        self.cmd_user = None
        self.have_cmd = False
        self.last_cmd_time = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_obs = False

        self.have_yaw_ref = False
        self.yaw_ref = 0.0

        self.path_locked = False
        self.x_ref = 0.0
        self.y_ref = 0.0

        self.dir_world_x = 1.0
        self.dir_world_y = 0.0
        self.normal_world_x = 0.0
        self.normal_world_y = 1.0

        self.last_body_dir_x = 1.0
        self.last_body_dir_y = 0.0
        self.have_last_body_dir = False

        self.hold_locked = False
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_yaw = 0.0

        self.was_turning = False
        self.timeout_printed = False

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/cmd_vel_user", Twist, self.cmd_cb, queue_size=10)
        rospy.Subscriber("/legged_robot_mpc_observation", mpc_observation, self.obs_cb, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_cb)

        rospy.loginfo("================================================")
        rospy.loginfo("[isaac_gazebo_like_cmd_filter v6] started")
        rospy.loginfo("[v6] subscribe: /cmd_vel_user")
        rospy.loginfo("[v6] publish:   /cmd_vel ONLY when cmd_vel_user is fresh")
        rospy.loginfo("[v6] NO AUTO ZERO PUBLISH, so robot will not stand by itself")
        rospy.loginfo("[v6] STAND/HOLD only when linear.z > %.2f", STAND_Z_THRESHOLD)
        rospy.loginfo("================================================")

    def cmd_cb(self, msg):
        self.cmd_user = msg
        self.have_cmd = True
        self.last_cmd_time = time.time()
        self.timeout_printed = False

    def obs_cb(self, msg):
        s = msg.state.value
        if len(s) < 12:
            return

        self.x = float(s[6])
        self.y = float(s[7])
        self.yaw = float(s[9])
        self.have_obs = True

        if not self.have_yaw_ref:
            self.yaw_ref = self.yaw
            self.have_yaw_ref = True
            rospy.loginfo(
                "[v6] yaw_ref locked from first obs: %.4f rad %.2f deg",
                self.yaw_ref,
                math.degrees(self.yaw_ref),
            )

    def command_is_fresh(self):
        if not self.have_cmd or self.last_cmd_time is None:
            return False
        return (time.time() - self.last_cmd_time) <= COMMAND_TIMEOUT_SEC

    def lock_hold_here(self):
        self.hold_x = self.x
        self.hold_y = self.y
        self.hold_yaw = self.yaw
        self.hold_locked = True

        rospy.loginfo(
            "[v6] HOLD locked: x=%.3f y=%.3f yaw=%.2f deg",
            self.hold_x,
            self.hold_y,
            math.degrees(self.hold_yaw),
        )

    def unlock_hold(self):
        if self.hold_locked:
            rospy.loginfo("[v6] HOLD unlocked")
        self.hold_locked = False

    def lock_path_here(self, vx_user, vy_user):
        speed_xy = math.sqrt(vx_user * vx_user + vy_user * vy_user)
        if speed_xy < 1e-9:
            return

        self.path_locked = True
        self.x_ref = self.x
        self.y_ref = self.y

        bx = vx_user / speed_xy
        by = vy_user / speed_xy

        self.last_body_dir_x = bx
        self.last_body_dir_y = by
        self.have_last_body_dir = True

        self.dir_world_x, self.dir_world_y = rot2d(self.yaw_ref, bx, by)
        self.normal_world_x = -self.dir_world_y
        self.normal_world_y = self.dir_world_x

        rospy.loginfo(
            "[v6] PATH locked: x_ref=%.3f y_ref=%.3f yaw_ref=%.2f deg body_dir=[%.2f %.2f] world_dir=[%.3f %.3f]",
            self.x_ref,
            self.y_ref,
            math.degrees(self.yaw_ref),
            bx,
            by,
            self.dir_world_x,
            self.dir_world_y,
        )

    def unlock_path(self):
        if self.path_locked:
            rospy.loginfo("[v6] PATH unlocked")
        self.path_locked = False
        self.have_last_body_dir = False

    def command_direction_changed(self, vx_user, vy_user):
        speed_xy = math.sqrt(vx_user * vx_user + vy_user * vy_user)
        if speed_xy < MIN_SPEED_TO_LOCK_PATH:
            return False

        bx = vx_user / speed_xy
        by = vy_user / speed_xy

        if not self.have_last_body_dir:
            return True

        dot = bx * self.last_body_dir_x + by * self.last_body_dir_y
        return dot < RELOCK_DIR_DOT

    def timer_cb(self, _event):
        # ============================================================
        # CRITICAL FIX:
        # If no fresh /cmd_vel_user exists, DO NOT publish /cmd_vel.
        # ============================================================
        if not self.command_is_fresh():
            self.unlock_path()
            self.unlock_hold()

            if not self.timeout_printed:
                rospy.logwarn("[v6] no fresh /cmd_vel_user -> publishing nothing")
                self.timeout_printed = True

            return

        if self.cmd_user is None:
            return

        cmd = self.cmd_user

        out = Twist()
        out.linear.x = cmd.linear.x
        out.linear.y = cmd.linear.y
        out.linear.z = cmd.linear.z

        out.angular.x = cmd.angular.x
        out.angular.y = cmd.angular.y
        out.angular.z = cmd.angular.z

        if not self.have_obs or not self.have_yaw_ref:
            self.pub.publish(out)
            return

        vx_user = cmd.linear.x
        vy_user = cmd.linear.y
        wz_user = cmd.angular.z
        z_user = cmd.linear.z

        speed_xy = math.sqrt(vx_user * vx_user + vy_user * vy_user)
        user_is_turning = abs(wz_user) > TURN_CMD_THRESHOLD
        stand_hold_requested = z_user > STAND_Z_THRESHOLD

        # ============================================================
        # Case A: yaw rotation command
        # ============================================================
        if user_is_turning:
            self.unlock_path()
            self.unlock_hold()
            self.was_turning = True

            rospy.loginfo_throttle(
                0.5,
                "[v6 TURN] pass-through wz=%.3f yaw=%.2f deg",
                out.angular.z,
                math.degrees(self.yaw),
            )

            self.pub.publish(out)
            return

        if self.was_turning and not user_is_turning:
            self.yaw_ref = self.yaw
            self.was_turning = False
            self.unlock_path()
            rospy.loginfo("[v6] finished turning, new yaw_ref=%.2f deg", math.degrees(self.yaw_ref))

        # ============================================================
        # Case B: explicit stand/hold command by linear.z
        # ============================================================
        if stand_hold_requested:
            self.unlock_path()

            if USE_STAND_HOLD:
                if not self.hold_locked:
                    self.lock_hold_here()

                ex = self.hold_x - self.x
                ey = self.hold_y - self.y

                v_world_x = K_HOLD_POS * ex
                v_world_y = K_HOLD_POS * ey
                v_world_x, v_world_y = limit_vec2(v_world_x, v_world_y, MAX_HOLD_V)

                v_body_x, v_body_y = rot2d(-self.yaw, v_world_x, v_world_y)

                yaw_err = wrap_pi(self.hold_yaw - self.yaw)
                wz_corr = clamp(K_HOLD_YAW * yaw_err, -MAX_HOLD_WZ, MAX_HOLD_WZ)

                out.linear.x = v_body_x
                out.linear.y = v_body_y
                out.linear.z = 0.0
                out.angular.z = wz_corr

                rospy.loginfo_throttle(
                    0.5,
                    "[v6 HOLD] err_xy=[%.3f %.3f] v_body=[%.3f %.3f] yaw=%.2f hold=%.2f wz=%.3f",
                    ex,
                    ey,
                    v_body_x,
                    v_body_y,
                    math.degrees(self.yaw),
                    math.degrees(self.hold_yaw),
                    wz_corr,
                )

            self.pub.publish(out)
            return

        # ============================================================
        # Case C: zero command but fresh
        # Publish zero only because user explicitly sent zero.
        # No hold.
        # ============================================================
        if speed_xy < MIN_SPEED_TO_LOCK_PATH:
            self.unlock_path()
            self.unlock_hold()

            out.linear.x = 0.0
            out.linear.y = 0.0
            out.linear.z = 0.0
            out.angular.z = 0.0

            rospy.loginfo_throttle(0.5, "[v6 ZERO] explicit zero command -> publishing zero, no hold")
            self.pub.publish(out)
            return

        # ============================================================
        # Case D: linear walking command
        # ============================================================
        self.unlock_hold()

        if (not self.path_locked) or self.command_direction_changed(vx_user, vy_user):
            self.lock_path_here(vx_user, vy_user)

        yaw_err = wrap_pi(self.yaw_ref - self.yaw)
        wz_corr = clamp(K_YAW * yaw_err, -MAX_WZ_CORR, MAX_WZ_CORR)
        out.angular.z += wz_corr

        cte = 0.0
        v_corr_body_x = 0.0
        v_corr_body_y = 0.0

        if USE_CROSSTRACK_CORRECTION and self.path_locked:
            dx = self.x - self.x_ref
            dy = self.y - self.y_ref

            cte = dx * self.normal_world_x + dy * self.normal_world_y

            v_corr_world_x = -K_CROSSTRACK * cte * self.normal_world_x
            v_corr_world_y = -K_CROSSTRACK * cte * self.normal_world_y
            v_corr_world_x, v_corr_world_y = limit_vec2(v_corr_world_x, v_corr_world_y, MAX_V_CORR)

            v_corr_body_x, v_corr_body_y = rot2d(-self.yaw, v_corr_world_x, v_corr_world_y)

            out.linear.x += v_corr_body_x
            out.linear.y += v_corr_body_y
            out.linear.z = 0.0

        rospy.loginfo_throttle(
            0.5,
            "[v6 WALK] cmd=[%.3f %.3f %.3f] yaw=%.2f ref=%.2f err=%.2f wz_corr=%.3f | cte=%.3f v_corr_body=[%.3f %.3f] out=[%.3f %.3f %.3f]",
            vx_user,
            vy_user,
            wz_user,
            math.degrees(self.yaw),
            math.degrees(self.yaw_ref),
            math.degrees(yaw_err),
            wz_corr,
            cte,
            v_corr_body_x,
            v_corr_body_y,
            out.linear.x,
            out.linear.y,
            out.angular.z,
        )

        self.pub.publish(out)


if __name__ == "__main__":
    IsaacGazeboLikeCmdFilter()
    rospy.spin()
