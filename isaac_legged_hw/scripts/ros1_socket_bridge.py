#!/usr/bin/env python3

import os
import json
import socket
import threading
import queue
import time

import rospy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


HOST = os.environ.get("ROS2_SOCKET_HOST", "host.docker.internal")
PORT = int(os.environ.get("ROS2_SOCKET_PORT", "50055"))

RECONNECT_DELAY_SEC = 1.0
SOCKET_TIMEOUT_SEC = 1.0
OUT_QUEUE_SIZE = 2000
PRINT_STATUS_EVERY_SEC = 2.0

FORWARD_CONTACTS_FROM_SOCKET = False
FORWARD_CLOCK_PACKET_FROM_SOCKET = False
PUBLISH_CLOCK_FROM_ODOM_HEADER = True


class Ros1SocketBridge:
    def __init__(self):
        rospy.init_node("ros1_socket_bridge", anonymous=False)

        self.out_queue = queue.Queue(maxsize=OUT_QUEUE_SIZE)

        self.sock = None
        self.sock_lock = threading.Lock()
        self.connected = False

        self.last_cmd_time = None
        self.last_feedback_time = None
        self.last_clock_time = None

        self.cmd_count = 0
        self.rx_count = 0
        self.bad_packet_count = 0
        self.dropped_contact_count = 0
        self.dropped_clock_count = 0
        self.clock_from_odom_count = 0

        self.joint_state_pub = rospy.Publisher(
            "/isaac/joint_states",
            JointState,
            queue_size=10,
        )

        self.imu_pub = rospy.Publisher(
            "/isaac/imu",
            Imu,
            queue_size=10,
        )

        self.odom_pub = rospy.Publisher(
            "/isaac/odom",
            Odometry,
            queue_size=10,
        )

        self.contacts_pub = None
        if FORWARD_CONTACTS_FROM_SOCKET:
            self.contacts_pub = rospy.Publisher(
                "/isaac/contacts",
                Float64MultiArray,
                queue_size=10,
            )

        self.clock_pub = rospy.Publisher(
            "/clock",
            Clock,
            queue_size=10,
        )

        self.joint_cmd_sub = rospy.Subscriber(
            "/isaac/joint_cmd",
            Float64MultiArray,
            self.joint_cmd_cb,
            queue_size=20,
        )

        threading.Thread(
            target=self.connection_loop,
            daemon=True,
        ).start()

        rospy.loginfo("================================================")
        rospy.loginfo("[ROS1 SOCKET] ROS1 socket bridge started")
        rospy.loginfo("[ROS1 SOCKET] Connecting to ROS2 socket bridge at %s:%d", HOST, PORT)
        rospy.loginfo("[ROS1 SOCKET] ROS1 sub: /isaac/joint_cmd")
        rospy.loginfo("[ROS1 SOCKET] ROS1 pub: /isaac/joint_states /isaac/imu /isaac/odom /clock")
        rospy.loginfo("[ROS1 SOCKET] Contacts from socket forwarded: %s", FORWARD_CONTACTS_FROM_SOCKET)
        rospy.loginfo("[ROS1 SOCKET] ROS2 /clock packet forwarded: %s", FORWARD_CLOCK_PACKET_FROM_SOCKET)
        rospy.loginfo("[ROS1 SOCKET] ROS1 /clock from /isaac/odom header: %s", PUBLISH_CLOCK_FROM_ODOM_HEADER)
        rospy.loginfo("================================================")

    def stamp_from_dict(self, d):
        sec = int(d.get("sec", 0))
        nanosec = int(d.get("nanosec", 0))
        return rospy.Time(sec, nanosec)

    def fill_header(self, header, d):
        stamp = d.get("stamp", {})
        header.stamp = self.stamp_from_dict(stamp)
        header.frame_id = d.get("frame_id", "")

    def fill_vec3(self, target, d):
        target.x = float(d.get("x", 0.0))
        target.y = float(d.get("y", 0.0))
        target.z = float(d.get("z", 0.0))

    def fill_quat(self, target, d):
        target.x = float(d.get("x", 0.0))
        target.y = float(d.get("y", 0.0))
        target.z = float(d.get("z", 0.0))
        target.w = float(d.get("w", 1.0))

    def publish_clock_from_header_dict(self, header_dict):
        if not PUBLISH_CLOCK_FROM_ODOM_HEADER:
            return

        stamp_dict = header_dict.get("stamp", {})
        msg = Clock()
        msg.clock = self.stamp_from_dict(stamp_dict)
        self.clock_pub.publish(msg)

        self.clock_from_odom_count += 1
        self.last_clock_time = time.time()

    def joint_cmd_cb(self, msg):
        data = [float(x) for x in msg.data]

        if len(data) != 60:
            rospy.logwarn_throttle(
                1.0,
                "[ROS1 SOCKET] /isaac/joint_cmd has wrong length: %d, expected 60",
                len(data),
            )

        packet = {
            "topic": "/isaac/joint_cmd",
            "type": "std_msgs/Float64MultiArray",
            "data": data,
        }

        self.cmd_count += 1
        self.last_cmd_time = time.time()

        try:
            self.out_queue.put_nowait(packet)
        except queue.Full:
            try:
                self.out_queue.get_nowait()
            except queue.Empty:
                pass

            try:
                self.out_queue.put_nowait(packet)
            except queue.Full:
                pass

    def connection_loop(self):
        while not rospy.is_shutdown():
            s = None

            try:
                rospy.loginfo("[ROS1 SOCKET] Connecting to %s:%d ...", HOST, PORT)

                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(SOCKET_TIMEOUT_SEC)
                s.connect((HOST, PORT))
                s.settimeout(None)

                with self.sock_lock:
                    self.sock = s
                    self.connected = True

                rospy.loginfo("[ROS1 SOCKET] Connected to ROS2 socket bridge at %s:%d", HOST, PORT)

                send_thread = threading.Thread(
                    target=self.send_loop,
                    args=(s,),
                    daemon=True,
                )

                recv_thread = threading.Thread(
                    target=self.recv_loop,
                    args=(s,),
                    daemon=True,
                )

                status_thread = threading.Thread(
                    target=self.status_loop,
                    daemon=True,
                )

                send_thread.start()
                recv_thread.start()
                status_thread.start()

                recv_thread.join()

            except Exception as e:
                rospy.logwarn("[ROS1 SOCKET] Connection failed: %s", str(e))

            finally:
                with self.sock_lock:
                    if self.sock is s:
                        self.sock = None
                    self.connected = False

                if s is not None:
                    try:
                        s.shutdown(socket.SHUT_RDWR)
                    except Exception:
                        pass

                    try:
                        s.close()
                    except Exception:
                        pass

                rospy.logwarn("[ROS1 SOCKET] Disconnected. Retrying in %.1f sec...", RECONNECT_DELAY_SEC)
                time.sleep(RECONNECT_DELAY_SEC)

    def send_loop(self, s):
        while not rospy.is_shutdown():
            try:
                packet = self.out_queue.get(timeout=0.2)
                line = json.dumps(packet, separators=(",", ":")) + "\n"
                s.sendall(line.encode("utf-8"))

            except queue.Empty:
                continue

            except Exception as e:
                rospy.logwarn("[ROS1 SOCKET] Send loop stopped: %s", str(e))
                break

    def recv_loop(self, s):
        try:
            file_obj = s.makefile("r")
        except Exception as e:
            rospy.logwarn("[ROS1 SOCKET] Could not create socket file object: %s", str(e))
            return

        while not rospy.is_shutdown():
            try:
                line = file_obj.readline()

                if not line:
                    rospy.logwarn("[ROS1 SOCKET] Socket closed by ROS2 bridge")
                    break

                packet = json.loads(line)
                self.rx_count += 1
                self.last_feedback_time = time.time()

                self.handle_packet(packet)

            except Exception as e:
                self.bad_packet_count += 1
                rospy.logwarn_throttle(
                    1.0,
                    "[ROS1 SOCKET] Bad packet or receive error: %s",
                    str(e),
                )

    def handle_packet(self, packet):
        topic = packet.get("topic", "")

        if topic == "/isaac/joint_states":
            msg = JointState()

            self.fill_header(msg.header, packet.get("header", {}))

            msg.name = list(packet.get("name", []))
            msg.position = [float(x) for x in packet.get("position", [])]
            msg.velocity = [float(x) for x in packet.get("velocity", [])]
            msg.effort = [float(x) for x in packet.get("effort", [])]

            self.joint_state_pub.publish(msg)

        elif topic == "/isaac/imu":
            msg = Imu()

            self.fill_header(msg.header, packet.get("header", {}))

            self.fill_quat(msg.orientation, packet.get("orientation", {}))
            self.fill_vec3(msg.angular_velocity, packet.get("angular_velocity", {}))
            self.fill_vec3(msg.linear_acceleration, packet.get("linear_acceleration", {}))

            msg.orientation_covariance = [
                float(x) for x in packet.get("orientation_covariance", [0.0] * 9)
            ]

            msg.angular_velocity_covariance = [
                float(x) for x in packet.get("angular_velocity_covariance", [0.0] * 9)
            ]

            msg.linear_acceleration_covariance = [
                float(x) for x in packet.get("linear_acceleration_covariance", [0.0] * 9)
            ]

            self.imu_pub.publish(msg)

        elif topic == "/isaac/odom":
            msg = Odometry()

            self.fill_header(msg.header, packet.get("header", {}))
            msg.child_frame_id = packet.get("child_frame_id", "base")

            self.fill_vec3(msg.pose.pose.position, packet.get("position", {}))
            self.fill_quat(msg.pose.pose.orientation, packet.get("orientation", {}))

            self.fill_vec3(msg.twist.twist.linear, packet.get("linear", {}))
            self.fill_vec3(msg.twist.twist.angular, packet.get("angular", {}))

            msg.pose.covariance = [
                float(x) for x in packet.get("pose_covariance", [0.0] * 36)
            ]

            msg.twist.covariance = [
                float(x) for x in packet.get("twist_covariance", [0.0] * 36)
            ]

            self.odom_pub.publish(msg)

            # Critical fix:
            # ROS1 /clock follows Isaac odom header stamp.
            self.publish_clock_from_header_dict(packet.get("header", {}))

        elif topic == "/isaac/contacts":
            if FORWARD_CONTACTS_FROM_SOCKET and self.contacts_pub is not None:
                msg = Float64MultiArray()
                msg.data = [float(x) for x in packet.get("data", [])]
                self.contacts_pub.publish(msg)
            else:
                self.dropped_contact_count += 1
                rospy.logwarn_throttle(
                    2.0,
                    "[ROS1 SOCKET] Dropping /isaac/contacts from socket. Real contacts come from mode_to_contacts.py. dropped=%d",
                    self.dropped_contact_count,
                )
            return

        elif topic == "/isaac/contacts_debug":
            self.dropped_contact_count += 1
            return

        elif topic == "/clock":
            if FORWARD_CLOCK_PACKET_FROM_SOCKET:
                msg = Clock()
                msg.clock = self.stamp_from_dict(packet.get("clock", {}))
                self.clock_pub.publish(msg)
            else:
                self.dropped_clock_count += 1
                rospy.logwarn_throttle(
                    2.0,
                    "[ROS1 SOCKET] Dropping forwarded ROS2 /clock. ROS1 /clock comes from /isaac/odom header. dropped=%d",
                    self.dropped_clock_count,
                )
            return

        else:
            rospy.logwarn_throttle(
                2.0,
                "[ROS1 SOCKET] Unknown packet topic from ROS2 bridge: %s",
                topic,
            )

    def status_loop(self):
        while not rospy.is_shutdown():
            time.sleep(PRINT_STATUS_EVERY_SEC)

            with self.sock_lock:
                connected = self.connected

            if not connected:
                break

            now = time.time()

            cmd_age = None if self.last_cmd_time is None else now - self.last_cmd_time
            feedback_age = None if self.last_feedback_time is None else now - self.last_feedback_time
            clock_age = None if self.last_clock_time is None else now - self.last_clock_time

            rospy.loginfo(
                "[ROS1 SOCKET] status | connected=%s | queued=%d | cmd_count=%d | rx_count=%d | bad_packets=%d | dropped_contacts=%d | dropped_clock=%d | clock_from_odom=%d | last_cmd_age=%s | last_feedback_age=%s | last_clock_age=%s",
                connected,
                self.out_queue.qsize(),
                self.cmd_count,
                self.rx_count,
                self.bad_packet_count,
                self.dropped_contact_count,
                self.dropped_clock_count,
                self.clock_from_odom_count,
                "None" if cmd_age is None else "%.3f sec" % cmd_age,
                "None" if feedback_age is None else "%.3f sec" % feedback_age,
                "None" if clock_age is None else "%.3f sec" % clock_age,
            )


def main():
    Ros1SocketBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
