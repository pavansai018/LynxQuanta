#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys, termios, tty

BANNER = """
LYNX M20 Jazzy Teleop
---------------------------
W/S : Move Forward/Back
A/D : Spin Left/Right
Q/E : Side-Walk (Lateral)
K   : Stop Wheels

1   : Stand
2   : Sit
3   : Reset Legs
"""

class LynxTeleop(Node):
    def __init__(self):
        super().__init__('lynx_teleop')
        self.leg_pub = self.create_publisher(Float64MultiArray, '/leg_controller/commands', 10)
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/wheel_velocity_controller/commands', 10)
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def send_legs(self, h, t, c):
        msg = Float64MultiArray()
        msg.data = [h, t, c] * 4 # Hip, Thigh, Calf for all 4 legs
        self.leg_pub.publish(msg)

    def drive(self, v1, v2, v3, v4):
        msg = Float64MultiArray()
        msg.data = [float(v1), float(v2), float(v3), float(v4)]
        self.wheel_pub.publish(msg)

    def run(self):
        print(BANNER)
        while rclpy.ok():
            key = self.get_key()
            if key == 'w': self.drive(-10, 10, -10, 10)
            elif key == 's': self.drive(10, -10, 10, -10)
            elif key == 'a': self.drive(10, 10, 10, 10)
            elif key == 'd': self.drive(-10, -10, -10, -10)
            elif key == 'q': self.send_legs(0.4, 0.7, -1.4) # Side lean left
            elif key == 'e': self.send_legs(-0.4, 0.7, -1.4) # Side lean right
            elif key == '1': self.send_legs(0.0, 0.7, -1.4) # Stand
            elif key == '2': self.send_legs(0.0, 1.2, -2.4) # Sit
            elif key == 'k': self.drive(0,0,0,0)
            elif key == '\x03': break

def main():
    rclpy.init()
    LynxTeleop().run()
    rclpy.shutdown()

if __name__ == '__main__': main()