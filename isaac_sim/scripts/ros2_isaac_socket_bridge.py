#!/usr/bin/env python3

import json
import os
import socket
import threading
import queue

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


HOST = os.environ.get("ISAAC_SOCKET_BIND", "0.0.0.0")
PORT = int(os.environ.get("ISAAC_SOCKET_PORT", "50055"))


def stamp_to_dict(stamp):
    return {
        "sec": int(stamp.sec),
        "nanosec": int(stamp.nanosec),
    }


def header_to_dict(header):
    return {
        "stamp": stamp_to_dict(header.stamp),
        "frame_id": header.frame_id,
    }


def vec3_to_dict(v):
    return {
        "x": float(v.x),
        "y": float(v.y),
        "z": float(v.z),
    }


def quat_to_dict(q):
    return {
        "x": float(q.x),
        "y": float(q.y),
        "z": float(q.z),
        "w": float(q.w),
    }


class Ros2IsaacSocketBridge(Node):
    def __init__(self):
        super().__init__("ros2_isaac_socket_bridge")

        self.out_queue = queue.Queue(maxsize=2000)
        self.conn = None
        self.conn_lock = threading.Lock()

        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/isaac/joint_cmd",
            10,
        )

        self.create_subscription(
            JointState,
            "/isaac/joint_states",
            self.joint_state_cb,
            10,
        )

        self.create_subscription(
            Imu,
            "/isaac/imu",
            self.imu_cb,
            10,
        )

        self.create_subscription(
            Odometry,
            "/isaac/odom",
            self.odom_cb,
            10,
        )

        self.create_subscription(
            Float64MultiArray,
            "/isaac/contacts",
            self.contacts_cb,
            10,
        )

        self.create_subscription(
            Clock,
            "/clock",
            self.clock_cb,
            10,
        )

        threading.Thread(target=self.server_loop, daemon=True).start()

        self.get_logger().info(
            f"ROS 2 Isaac socket bridge listening on {HOST}:{PORT}"
        )

    def has_client(self):
        with self.conn_lock:
            return self.conn is not None

    def enqueue(self, packet):
        if not self.has_client():
            return

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

    def joint_state_cb(self, msg):
        packet = {
            "topic": "/isaac/joint_states",
            "type": "sensor_msgs/JointState",
            "header": header_to_dict(msg.header),
            "name": list(msg.name),
            "position": [float(x) for x in msg.position],
            "velocity": [float(x) for x in msg.velocity],
            "effort": [float(x) for x in msg.effort],
        }
        self.enqueue(packet)

    def imu_cb(self, msg):
        packet = {
            "topic": "/isaac/imu",
            "type": "sensor_msgs/Imu",
            "header": header_to_dict(msg.header),
            "orientation": quat_to_dict(msg.orientation),
            "angular_velocity": vec3_to_dict(msg.angular_velocity),
            "linear_acceleration": vec3_to_dict(msg.linear_acceleration),
            "orientation_covariance": [float(x) for x in msg.orientation_covariance],
            "angular_velocity_covariance": [float(x) for x in msg.angular_velocity_covariance],
            "linear_acceleration_covariance": [float(x) for x in msg.linear_acceleration_covariance],
        }
        self.enqueue(packet)

    def odom_cb(self, msg):
        packet = {
            "topic": "/isaac/odom",
            "type": "nav_msgs/Odometry",
            "header": header_to_dict(msg.header),
            "child_frame_id": msg.child_frame_id,
            "position": vec3_to_dict(msg.pose.pose.position),
            "orientation": quat_to_dict(msg.pose.pose.orientation),
            "linear": vec3_to_dict(msg.twist.twist.linear),
            "angular": vec3_to_dict(msg.twist.twist.angular),
            "pose_covariance": [float(x) for x in msg.pose.covariance],
            "twist_covariance": [float(x) for x in msg.twist.covariance],
        }
        self.enqueue(packet)

    def contacts_cb(self, msg):
        packet = {
            "topic": "/isaac/contacts",
            "type": "std_msgs/Float64MultiArray",
            "data": [float(x) for x in msg.data],
        }
        self.enqueue(packet)

    def clock_cb(self, msg):
        packet = {
            "topic": "/clock",
            "type": "rosgraph_msgs/Clock",
            "clock": stamp_to_dict(msg.clock),
        }
        self.enqueue(packet)

    def publish_joint_cmd(self, data):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in data]
        self.joint_cmd_pub.publish(msg)

    def server_loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((HOST, PORT))
        srv.listen(1)

        self.get_logger().info(f"Waiting for ROS 1 Docker client on port {PORT}")

        while rclpy.ok():
            conn, addr = srv.accept()

            with self.conn_lock:
                self.conn = conn

            self.get_logger().info(f"ROS 1 client connected from {addr}")

            send_thread = threading.Thread(
                target=self.send_loop,
                args=(conn,),
                daemon=True,
            )
            recv_thread = threading.Thread(
                target=self.recv_loop,
                args=(conn,),
                daemon=True,
            )

            send_thread.start()
            recv_thread.start()

            recv_thread.join()

            with self.conn_lock:
                if self.conn is conn:
                    self.conn = None

            try:
                conn.close()
            except Exception:
                pass

            self.get_logger().warn("ROS 1 client disconnected; waiting again")

    def send_loop(self, conn):
        while rclpy.ok():
            try:
                packet = self.out_queue.get(timeout=0.2)
                line = json.dumps(packet) + "\n"
                conn.sendall(line.encode("utf-8"))
            except queue.Empty:
                continue
            except Exception:
                break

    def recv_loop(self, conn):
        file_obj = conn.makefile("r")

        while rclpy.ok():
            line = file_obj.readline()

            if not line:
                break

            try:
                packet = json.loads(line)
            except Exception:
                continue

            if packet.get("topic") == "/isaac/joint_cmd":
                data = packet.get("data", [])
                self.publish_joint_cmd(data)


def main():
    rclpy.init()
    node = Ros2IsaacSocketBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
