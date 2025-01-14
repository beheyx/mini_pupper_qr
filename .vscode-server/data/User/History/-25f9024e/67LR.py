from networktables import NetworkTables
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import rclpy
import time
from networktables import NetworkTables

NetworkTables.initialize()
sd = NetworkTables.getTable("PositionTable")
pos_x = 0
pos_y = 0
pos_z = 0
goal_x = 0
goal_y = 0
goal_z = 0
z_angle = 0


class Driver(Node):
    def __init__(self):
        super().__init__('bridge') # type: ignore
        self.point = None
        self.current_point = None
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish(self, lx, ly, lz, ax, ay, az):
        msg = Twist()
        linear = Vector3()
        linear.x = float(lx)
        linear.y = float(ly)
        linear.z = float(lz)
        angular = Vector3()
        angular.x = float(ax)
        angular.y = float(ay)
        angular.z = float(az)
        msg.linear = linear
        msg.angular = angular
        self.publisher_.publish(msg)

rclpy.init()
driver_node = Driver()

try:
    while True:
        print(time.monotonic())
        lx = sd.getNumber("lx", 0)
        ly = sd.getNumber("ly", 0)
        lz = sd.getNumber("lz", 0)
        ax = sd.getNumber("ax", 0)
        ay = sd.getNumber("ay", 0)
        az = sd.getNumber("az", 0)
        driver_node.publish(lx, ly, lz, ax, ay, az)
        time.sleep(.1)
except KeyboardInterrupt:
    pass
    

driver_node.destroy_node()
rclpy.shutdown()