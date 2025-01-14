from networktables import NetworkTables
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector
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
