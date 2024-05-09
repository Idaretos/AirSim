import airsim
import cv2
import numpy as np
import os
import pprint 
import tempfile
import time

# Connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

client.enableApiControl(True, 'drone_1')
client.armDisarm(True, 'drone_1')

# Takeoff command must be given before the flight command
client.takeoffAsync(vehicle_name='drone_1')

print(time.time())
client.moveToPositionAsync(0, 3, -5, 3, vehicle_name='drone_1').join()
client.moveToPositionAsync(3, 0, -7, 3, vehicle_name='drone_1').join()
client.moveToPositionAsync(0, -3, -8, 3, vehicle_name='drone_1').join()

# Finish control
airsim.wait_key('Press any key to finish')
client.armDisarm(False, 'drone_1')
client.reset()
client.enableApiControl(False, 'drone_1')