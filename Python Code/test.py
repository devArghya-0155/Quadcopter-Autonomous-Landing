import cv2
import numpy as np
import sim
import time

from states import State, SimInit, Drone, Marker, TrajectoryGenerator, ObjectDetection
import utils

# clientID, searchAngle, pidx, pidy =  SimInit(), 0.610865, [0.1, 0.1, 0.5], [0.1, 0.1, 0.5]

clientID = SimInit()
    
marker = Marker()
drone = Drone()

drone.startSim()
start = time.time()


obj = ObjectDetection(drone)

while True:

    obj.detection(drone, marker)
