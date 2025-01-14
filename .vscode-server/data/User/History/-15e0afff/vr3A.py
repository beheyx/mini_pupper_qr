from networktables import NetworkTables
import numpy as np
import movement_api as ma
import time

NetworkTables.initialize(server='192.168.1.109')
nt = NetworkTables.getTable("localization")

ma.look_up()
ma.move_forward()
time.sleep(5)
ma.stop()