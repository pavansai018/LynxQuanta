import socket
import struct
import pickle
import time
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


LAPTOP_IP = os.environ.get("LAPTOP_IP", "10.21.31.100")
PORT = int(os.environ.get("LAPTOP_PORT", "50060"))


class RobotFullPointCloudSender(Node):
    def __init__(self):
        super().__init__("robot_full_pc2_tcp_sender")

        self.sock = None

        self.sent_counts = {
            "/LIDAR/POINTS": 0,
            "/LIDAR/POINTS2": 0,
        }

        self.last_sent_time = {
            "/LIDAR/POINTS": 0.0,
            "/LIDAR/POINTS2": 0.0,
        }

        # Full frames are sent.
        # This only caps frequency, not point count.
        # 0.0 = no throttle.
        # Start with 0.2 if full-rate overloads LAN/laptop.
        self.min_period = float(os.environ.get("PC2_MIN_PERIOD", "0.0"))

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub1 = self.create_subscription(
            PointCloud2,
            "/LIDAR/POINTS",
            lambda msg: self.cb("/LIDAR/POINTS", msg),
            qos,
        )

        self.sub2 = self.create_subscription(
            PointCloud2,
            "/LIDAR/POINTS2",
            lambda msg: self.cb("/LIDAR/POINTS2", msg),
            qos,
        )

        self.get_logger().info("subscribed to /LIDAR/POINTS and /LIDAR/POINTS2")
        self.get_logger().info(f"target laptop TCP: {LAPTOP_IP}:{PORT}")
        self.get_logger().info(f"PC2_MIN_PERIOD={self.min_period}")

    def connect_laptop(self):
        while rclpy.ok() and self.sock is None:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.connect((LAPTOP_IP, PORT))
                self.sock = s
                self.get_logger().info(f"connected to laptop {LAPTOP_IP}:{PORT}")

            except Exception as e:
                self.get_logger().warn(f"waiting for laptop receiver: {e}")
                time.sleep(1.0)

    def make_packet(self, topic, msg):
        return {
            "topic": topic,

            "sec": msg.header.stamp.sec,
            "nanosec": msg.header.stamp.nanosec,
            "frame_id": msg.header.frame_id,

            "height": msg.height,
            "width": msg.width,

            "fields": [
                {
                    "name": f.name,
                    "offset": f.offset,
                    "datatype": f.datatype,
                    "count": f.count,
                }
                for f in msg.fields
            ],

            "is_bigendian": msg.is_bigendian,
            "point_step": msg.point_step,
            "row_step": msg.row_step,

            # Full cloud data. No downsampling.
            "data": bytes(msg.data),

            "is_dense": msg.is_dense,
        }

    def cb(self, topic, msg):
        now = time.time()

        if self.min_period > 0.0:
            if now - self.last_sent_time[topic] < self.min_period:
                return

            self.last_sent_time[topic] = now

        if self.sock is None:
            self.connect_laptop()

        pkt = self.make_packet(topic, msg)

        try:
            payload = pickle.dumps(pkt, protocol=pickle.HIGHEST_PROTOCOL)
            header = struct.pack("!I", len(payload))

            self.sock.sendall(header + payload)

            self.sent_counts[topic] += 1

            if self.sent_counts[topic] % 5 == 0:
                self.get_logger().info(
                    f"sent {topic}: "
                    f"count={self.sent_counts[topic]}, "
                    f"width={msg.width}, "
                    f"bytes={len(msg.data)}, "
                    f"frame={msg.header.frame_id}"
                )

        except Exception as e:
            self.get_logger().warn(f"TCP send failed, reconnecting: {e}")

            try:
                self.sock.close()
            except Exception:
                pass

            self.sock = None


def main():
    rclpy.init()
    node = RobotFullPointCloudSender()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
