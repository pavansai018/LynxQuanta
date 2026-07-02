import socket
import struct
import pickle
import array

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


HOST = "0.0.0.0"
PORT = 50060


class LaptopFullPointCloudReceiver(Node):
    def __init__(self):
        super().__init__("laptop_full_pc2_receiver")

        # Do NOT call this self.publishers.
        # rclpy.Node already has an internal read-only property with that name.
        self.pc2_pubs = {
            "/LIDAR/POINTS": self.create_publisher(
                PointCloud2,
                "/LIDAR/POINTS_TCP",
                10,
            ),
            "/LIDAR/POINTS2": self.create_publisher(
                PointCloud2,
                "/LIDAR/POINTS2_TCP",
                10,
            ),
        }

        self.counts = {
            "/LIDAR/POINTS": 0,
            "/LIDAR/POINTS2": 0,
        }

    def make_msg(self, pkt):
        msg = PointCloud2()

        msg.header.stamp.sec = pkt["sec"]
        msg.header.stamp.nanosec = pkt["nanosec"]
        msg.header.frame_id = pkt["frame_id"]

        msg.height = pkt["height"]
        msg.width = pkt["width"]

        msg.fields = []
        for f in pkt["fields"]:
            pf = PointField()
            pf.name = f["name"]
            pf.offset = f["offset"]
            pf.datatype = f["datatype"]
            pf.count = f["count"]
            msg.fields.append(pf)

        msg.is_bigendian = pkt["is_bigendian"]
        msg.point_step = pkt["point_step"]
        msg.row_step = pkt["row_step"]

        # uint8[] data field
        msg.data = array.array("B", pkt["data"])

        msg.is_dense = pkt["is_dense"]

        return msg

    def publish_packet(self, pkt):
        src_topic = pkt["topic"]

        if src_topic not in self.pc2_pubs:
            self.get_logger().warn(f"unknown source topic received: {src_topic}")
            return

        msg = self.make_msg(pkt)
        self.pc2_pubs[src_topic].publish(msg)

        self.counts[src_topic] += 1

        if self.counts[src_topic] % 5 == 0:
            if src_topic == "/LIDAR/POINTS":
                dst_topic = "/LIDAR/POINTS_TCP"
            else:
                dst_topic = "/LIDAR/POINTS2_TCP"

            self.get_logger().info(
                f"{src_topic} -> {dst_topic}: "
                f"count={self.counts[src_topic]}, "
                f"width={msg.width}, "
                f"bytes={len(msg.data)}, "
                f"frame={msg.header.frame_id}"
            )


def recv_exact(conn, n):
    buf = b""

    while len(buf) < n:
        chunk = conn.recv(n - len(buf))

        if not chunk:
            raise ConnectionError("socket closed")

        buf += chunk

    return buf


def main():
    rclpy.init()
    node = LaptopFullPointCloudReceiver()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(1)

    node.get_logger().info(f"listening on {HOST}:{PORT}")

    try:
        while rclpy.ok():
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            node.get_logger().info(f"robot connected from {addr}")

            try:
                while rclpy.ok():
                    size_buf = recv_exact(conn, 4)
                    size = struct.unpack("!I", size_buf)[0]

                    payload = recv_exact(conn, size)
                    pkt = pickle.loads(payload)

                    node.publish_packet(pkt)
                    rclpy.spin_once(node, timeout_sec=0.0)

            except Exception as e:
                node.get_logger().warn(f"connection dropped: {e}")

                try:
                    conn.close()
                except Exception:
                    pass

    except KeyboardInterrupt:
        pass

    finally:
        try:
            srv.close()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
