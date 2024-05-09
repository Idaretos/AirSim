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

# Start control
drone_names = ["Drone" + str(i) for i in range(40)]
for drone in drone_names:
    client.enableApiControl(True, drone)
for drone in drone_names:
    client.armDisarm(True, drone)

# Takeoff command must be given before the flight command
# airsim.wait_key('Press any key to takeoff')
takeoff_tasks = [client.takeoffAsync(vehicle_name=drone) for drone in drone_names]
for task in takeoff_tasks:
    task.join()

time.sleep(1)

for i in range(100):
    # Flight control command
    # airsim.wait_key('Press any key to move vehicles')
    move_tasks = [client.moveToPositionAsync(0, 0, -int(drone[-1])-10, 5, vehicle_name=drone) for drone in drone_names]
    for task in move_tasks:
        task.join()

    time.sleep(1)

    # Flight control command
    # airsim.wait_key('Press any key to move vehicles')
    move_tasks = [client.moveToPositionAsync(0, 0, -25, 5, vehicle_name=drone) for drone in drone_names]
    for task in move_tasks:
        task.join()

    time.sleep(1)

    # Flight control command
    # airsim.wait_key('Press any key to move vehicles')
    move_tasks = [client.moveToPositionAsync(0, 0, -5, 5, vehicle_name=drone) for drone in drone_names]
    for task in move_tasks:
        task.join()

    time.sleep(1)

time.sleep(5)

# Finish control
for drone in drone_names:
    client.armDisarm(False, drone)
client.reset()
for drone in drone_names:
    client.enableApiControl(False, drone)