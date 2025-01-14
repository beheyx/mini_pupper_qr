from networktables import NetworkTables
import numpy as np
import movement_api as ma
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
import rclpy

NetworkTables.initialize(server='192.168.1.109')
nt = NetworkTables.getTable("localization")

ma.move_forward()
