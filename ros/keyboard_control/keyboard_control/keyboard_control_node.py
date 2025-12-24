#!/usr/bin/env python3
import sys, termios, tty, select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_mavros')
        self.pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.get_logger().info("Keyboard Teleop Ready. Use WASD + QE. CTRL-C to exit.")

        # Save original terminal settings
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)

        # Switch to raw mode
        tty.setcbreak(self.fd)

    def restore_terminal(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        """Non-blocking key read"""
        dr, _, _ = select.select([sys.stdin], [], [], 0.05)
        if dr:
            return sys.stdin.read(1)
        return None

    def run(self):
        twist = Twist()
        try:
            while rclpy.ok():
                key = self.get_key()
                if key is None:
                    continue

                twist = Twist()

                if key == 'w': twist.linear.x = 1.0
                elif key == 's': twist.linear.x = -1.0
                elif key == 'a': twist.linear.y = 1.0
                elif key == 'd': twist.linear.y = -1.0
                elif key == 'q': twist.angular.z = 1.0
                elif key == 'e': twist.angular.z = -1.0
                elif key == ' ': pass  # stop
                else: continue

                self.pub.publish(twist)
                self.get_logger().info(f"Sent: {twist}")
        finally:
            # **Always restore terminal even on Ctrl-C**
            self.restore_terminal()
            self.get_logger().info("Exiting safely…")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
