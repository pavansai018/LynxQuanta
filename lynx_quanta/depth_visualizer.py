import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2


class DepthVisualizer(Node):
    def __init__(self):
        super().__init__('depth_visualizer')
        self.bridge = CvBridge()

        self.sub_front = self.create_subscription(
            Image,
            '/camera_front/depth_image',
            self.callback_front,
            10
        )

        self.pub_front = self.create_publisher(
            Image,
            '/camera_front/depth_vis',
            10
        )

        self.sub_rear = self.create_subscription(
            Image,
            '/camera_rear/depth_image',
            self.callback_rear,
            10
        )

        self.pub_rear = self.create_publisher(
            Image,
            '/camera_rear/depth_vis',
            10
        )

    def callback_front(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)

        min_depth = 0.1
        max_depth = 10.0

        depth_clipped = np.clip(depth, min_depth, max_depth)

        depth_norm = ((depth_clipped - min_depth) / (max_depth - min_depth) * 255.0).astype(np.uint8)

        depth_colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

        out_msg = self.bridge.cv2_to_imgmsg(depth_colored, encoding='bgr8')
        out_msg.header = msg.header
        self.pub_front.publish(out_msg)

    def callback_rear(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)

        min_depth = 0.1
        max_depth = 10.0

        depth_clipped = np.clip(depth, min_depth, max_depth)

        depth_norm = ((depth_clipped - min_depth) / (max_depth - min_depth) * 255.0).astype(np.uint8)

        depth_colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

        out_msg = self.bridge.cv2_to_imgmsg(depth_colored, encoding='bgr8')
        out_msg.header = msg.header
        self.pub_rear.publish(out_msg)


def main():
    rclpy.init()
    node = DepthVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()