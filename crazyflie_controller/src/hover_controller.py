#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import math

class PositionController(Node):
    def __init__(self):
        super().__init__('position_controller')

        # Parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 1.0)

        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value

        # State variables
        self.prev_error = [0.0, 0.0, 0.0]
        self.prev_time = self.get_clock().now()

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

        # Target
        self.target = [
            self.get_parameter('target_x').get_parameter_value().double_value,
            self.get_parameter('target_y').get_parameter_value().double_value,
            self.get_parameter('target_z').get_parameter_value().double_value,
        ]

        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.get_logger().info('✅ Position controller initialized')

    def odom_callback(self, msg: Odometry):
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        self.current_position[2] = msg.pose.pose.position.z

    def control_loop(self):
        # Compute position error
        error = [
            self.target[0] - self.current_position[0],
            self.target[1] - self.current_position[1],
            self.target[2] - self.current_position[2]
        ]

        # Compute derivative
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt == 0.0:
            return
        d_error = [(error[i] - self.prev_error[i]) / dt for i in range(3)]

        # PID control (only P + D for simplicity)
        cmd = Twist()
        cmd.linear.x = self.kp * error[0] + self.kd * d_error[0]
        cmd.linear.y = self.kp * error[1] + self.kd * d_error[1]
        cmd.linear.z = self.kp * error[2] + self.kd * d_error[2]

        # Clamp velocities for stability
        cmd.linear.x = max(min(cmd.linear.x, 1.0), -1.0)
        cmd.linear.y = max(min(cmd.linear.y, 1.0), -1.0)
        cmd.linear.z = max(min(cmd.linear.z, 2.0), -2.0)

        self.cmd_pub.publish(cmd)

        # Store
        self.prev_error = error
        self.prev_time = now

def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
