#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from sensor_msgs_py import point_cloud2

import tf2_ros
from tf_transformations import quaternion_matrix


class DualLidarMerger(Node):
    def __init__(self):
        super().__init__('dual_lidar_merger')

        self.target_frame = self.declare_parameter('target_frame', 'base_link').value

        self.front_topic = self.declare_parameter('front_topic', '/lidar_front/points').value
        self.rear_topic = self.declare_parameter('rear_topic', '/lidar_rear/points').value

        self.merged_cloud_topic = self.declare_parameter(
            'merged_cloud_topic', '/lidar_merged_points'
        ).value

        self.merged_scan_topic = self.declare_parameter(
            'merged_scan_topic', '/scan'
        ).value

        # Height slice used for 2D scan projection from 3D cloud
        self.min_height = float(self.declare_parameter('min_height', -0.20).value)
        self.max_height = float(self.declare_parameter('max_height', 0.40).value)

        self.angle_min = float(self.declare_parameter('angle_min', -math.pi).value)
        self.angle_max = float(self.declare_parameter('angle_max', math.pi).value)
        self.angle_increment = float(self.declare_parameter('angle_increment', 0.00349).value)

        self.range_min = float(self.declare_parameter('range_min', 0.15).value)
        self.range_max = float(self.declare_parameter('range_max', 50.0).value)

        self.publish_rate = float(self.declare_parameter('publish_rate', 10.0).value)

        self.front_cloud = None
        self.rear_cloud = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.front_sub = self.create_subscription(
            PointCloud2,
            self.front_topic,
            self.front_callback,
            10
        )

        self.rear_sub = self.create_subscription(
            PointCloud2,
            self.rear_topic,
            self.rear_callback,
            10
        )

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            self.merged_cloud_topic,
            10
        )

        self.scan_pub = self.create_publisher(
            LaserScan,
            self.merged_scan_topic,
            10
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_outputs
        )

        self.get_logger().info('Dual lidar merger started')
        self.get_logger().info(f'Front cloud: {self.front_topic}')
        self.get_logger().info(f'Rear cloud : {self.rear_topic}')
        self.get_logger().info(f'3D output  : {self.merged_cloud_topic}')
        self.get_logger().info(f'2D output  : {self.merged_scan_topic}')
        self.get_logger().info(f'Target frame: {self.target_frame}')

    def front_callback(self, msg):
        self.front_cloud = msg

    def rear_callback(self, msg):
        self.rear_cloud = msg

    def cloud_to_xyz(self, cloud_msg):
        points = []

        for p in point_cloud2.read_points(
            cloud_msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):
            x = float(p[0])
            y = float(p[1])
            z = float(p[2])

            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append([x, y, z])

        if not points:
            return np.empty((0, 3), dtype=np.float32)

        return np.asarray(points, dtype=np.float32)

    def transform_points_to_target(self, points, source_frame):
        if points.shape[0] == 0:
            return points

        try:
            # Use latest TF. This avoids timestamp mismatch problems in simulation.
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.2)
            )
        except Exception as e:
            self.get_logger().warn(
                f'No TF from {source_frame} to {self.target_frame}: {e}',
                throttle_duration_sec=1.0
            )
            return None

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation

        mat = quaternion_matrix([q.x, q.y, q.z, q.w])
        rot = mat[:3, :3]
        trans = np.array([t.x, t.y, t.z], dtype=np.float32)

        return (points @ rot.T + trans).astype(np.float32)

    def publish_outputs(self):
        if self.front_cloud is None or self.rear_cloud is None:
            return

        front_time = self.front_cloud.header.stamp.sec + self.front_cloud.header.stamp.nanosec * 1e-9
        rear_time = self.rear_cloud.header.stamp.sec + self.rear_cloud.header.stamp.nanosec * 1e-9

        if abs(front_time - rear_time) > 0.12:
            return

        front_frame = self.front_cloud.header.frame_id
        rear_frame = self.rear_cloud.header.frame_id

        front_points = self.cloud_to_xyz(self.front_cloud)
        rear_points = self.cloud_to_xyz(self.rear_cloud)

        front_base = self.transform_points_to_target(front_points, front_frame)
        rear_base = self.transform_points_to_target(rear_points, rear_frame)

        if front_base is None or rear_base is None:
            return

        if front_base.shape[0] == 0 and rear_base.shape[0] == 0:
            return

        merged_points = np.vstack((front_base, rear_base)).astype(np.float32)
        merged_points = self.remove_robot_body_points(merged_points)

        # now = self.get_clock().now().to_msg()

        # self.publish_merged_cloud(merged_points, now)
        # self.publish_merged_scan(merged_points, now)
        front_time = self.front_cloud.header.stamp.sec + self.front_cloud.header.stamp.nanosec * 1e-9
        rear_time = self.rear_cloud.header.stamp.sec + self.rear_cloud.header.stamp.nanosec * 1e-9

        if front_time >= rear_time:
            stamp = self.front_cloud.header.stamp
        else:
            stamp = self.rear_cloud.header.stamp

        self.publish_merged_cloud(merged_points, stamp)
        self.publish_merged_scan(merged_points, stamp)

    def remove_robot_body_points(self, points):
        if points.shape[0] == 0:
            return points

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        # Approx robot body/self-filter box in base_link frame.
        # Slightly bigger than base collision.
        inside_robot_body = (
            (x > -0.45) & (x < 0.45) &
            (y > -0.18) & (y < 0.18) &
            (z > -0.12) & (z < 0.25)
        )

        return points[~inside_robot_body]
    
    def publish_merged_cloud(self, merged_points, stamp):
        header = self.front_cloud.header
        header.stamp = stamp
        header.frame_id = self.target_frame

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = point_cloud2.create_cloud(
            header,
            fields,
            merged_points.tolist()
        )

        self.cloud_pub.publish(cloud_msg)

    def publish_merged_scan(self, merged_points, stamp):
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.target_frame

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.publish_rate
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        num_bins = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
        ranges = np.full(num_bins, np.inf, dtype=np.float32)

        x = merged_points[:, 0]
        y = merged_points[:, 1]
        z = merged_points[:, 2]

        height_mask = (z >= self.min_height) & (z <= self.max_height)

        x = x[height_mask]
        y = y[height_mask]

        if x.shape[0] > 0:
            angles = np.arctan2(y, x)
            distances = np.sqrt(x * x + y * y)

            valid = (
                (angles >= self.angle_min) &
                (angles <= self.angle_max) &
                (distances >= self.range_min) &
                (distances <= self.range_max)
            )

            angles = angles[valid]
            distances = distances[valid]

            indices = ((angles - self.angle_min) / self.angle_increment).astype(np.int32)

            for idx, dist in zip(indices, distances):
                if 0 <= idx < num_bins:
                    if dist < ranges[idx]:
                        ranges[idx] = dist

        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = DualLidarMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()