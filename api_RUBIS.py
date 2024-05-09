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
drone_total = ["Drone" + str(i) for i in range(42)]
drone_R = [["Drone" + str(i) for i in range(9)], [30,30,30,30,30,30,30,30,30]]
drone_U = [["Drone" + str(i) for i in range(9, 16)], [30,30,30,-30,-30,-30,-30]]
drone_B = [["Drone" + str(i) for i in range(16, 25)], [-30,-30,-30,-30,-30,-30,-30, 30,30]]
drone_I = [["Drone" + str(i) for i in range(25, 33)], [30,30,30,30,30,30,30,30]]
drone_S = [["Drone" + str(i) for i in range(33, 42)], [-30,-30,-30,-30,-30,-30,-30,-30,-30]]

for drone in drone_total:
    client.enableApiControl(True, drone)
for drone in drone_total:
    client.armDisarm(True, drone)

# Takeoff command must be given before the flight command

takeoff_tasks = [client.takeoffAsync(vehicle_name=drone) for drone in drone_total]
for task in takeoff_tasks:
    task.join()


# Flight control command
move_tasks = [client.moveToPositionAsync(0, drone_R[1][i]/3, -5, 5, vehicle_name=drone_R[0][i]) for i in range(len(drone_R[0]))]
# for task in move_tasks:
# 	  task.join()
# time.sleep(1)
move_tasks = [client.moveToPositionAsync(0, drone_U[1][i]/3, -5, 5, vehicle_name=drone_U[0][i]) for i in range(len(drone_U[0]))]
# for task in move_tasks:
# 	  task.join()
# time.sleep(1)
move_tasks = [client.moveToPositionAsync(0, drone_B[1][i]/3, -5, 5, vehicle_name=drone_B[0][i]) for i in range(len(drone_B[0]))]
# for task in move_tasks:
# 	  task.join()
# time.sleep(1)
move_tasks = [client.moveToPositionAsync(0, drone_I[1][i]/3, -5, 5, vehicle_name=drone_I[0][i]) for i in range(len(drone_I[0]))]
# for task in move_tasks:
# 	  task.join()
# time.sleep(1)
move_tasks = [client.moveToPositionAsync(0, drone_S[1][i]/3, -5, 5, vehicle_name=drone_S[0][i]) for i in range(len(drone_S[0]))]


move_tasks = [client.moveToPositionAsync(0, 2*drone_R[1][i]/3, 2.5, 5, vehicle_name=drone_R[0][i]) for i in range(len(drone_R[0]))]

move_tasks = [client.moveToPositionAsync(0, 2*drone_U[1][i]/3, 2.5, 5, vehicle_name=drone_U[0][i]) for i in range(len(drone_U[0]))]

move_tasks = [client.moveToPositionAsync(0, 2*drone_B[1][i]/3, 2.5, 5, vehicle_name=drone_B[0][i]) for i in range(len(drone_B[0]))]

move_tasks = [client.moveToPositionAsync(0, 2*drone_I[1][i]/3, 2.5, 5, vehicle_name=drone_I[0][i]) for i in range(len(drone_I[0]))]

move_tasks = [client.moveToPositionAsync(0, 2*drone_S[1][i]/3, 2.5, 5, vehicle_name=drone_S[0][i]) for i in range(len(drone_S[0]))]

# Finish control
airsim.wait_key('Press any key to finish')
for drone in drone_total:
    client.armDisarm(False, drone)
client.reset()
for drone in drone_total:
    client.enableApiControl(False, drone)