from networktables import NetworkTables
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
import rclpy

NetworkTables.initialize(server='192.168.1.109')
nt = NetworkTables.getTable("localization")
