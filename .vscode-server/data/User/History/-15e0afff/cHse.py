from networktables import NetworkTables
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import rclpy

NetworkTables.initialize(server='192.168.1.109')
nt = NetworkTables.getTable("localization")

class Driver(Node):
    def __init__(self):
        super().__init__('bridge') # type: ignore
        self.point = None
        self.current_point = None
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish(self):
        msg = Twist()
        linear = Vector3()
        linear.x = 1.0
        linear.y = 0.0
        linear.z = 0.0
        angular = Vector3()
        angular.x = 0.0
        angular.y = 0.0
        angular.z = 0.0
        msg.linear = linear
        msg.angular = angular
        self.publisher_.publish(msg)

if __name__ == "__main__":
    rclpy.init()
    driver_node = Driver()
    driver_node.publish()
    rclpy.spin(driver_node)
    driver_node.destroy_node()
    rclpy.shutdown()