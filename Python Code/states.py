'''

class SimObj():
    def __init__(self):
        pass

    def setup(self):
        pass

    def loop(self):
        self.state_func()
        self.state_transition()
    
    def state_transition():
        if self.state == FLYING and marker === detected:
            self.state = CHASING
        elif self.state == CHASING and marker is below:
            self.state = LANDING
        
            

simobjs = [drone, camera, marker, drone2, car, robot, plotter]

while True:
    for obj in simobjs:
        obj.loop()

'''

import numpy as np
import cv2
from cv2 import aruco
from enum import Enum, auto
import sim
import sys
import time
import math
import signal

import quadrocoptertrajectory as quadtraj
import utils

clientID, searchAngle, pidx, pidy, w, h, pErrorx, pErrory =  0, 0.610865, [0.1, 0.1, 0.5], [0.1, 0.1, 0.5], 256, 256, 0, 0
dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)


def signal_handler(signal, frame):
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class State (Enum): #Possible Drone States
    IDLE = auto()
    TAKEOFF = auto()
    INITIAL_SEARCH = auto()
    ESTIMATED_SEARCH = auto()
    TRACK = auto()
    APPROACH = auto()
    LAND = auto()


class SimInit():
    def __init__(self) -> None:
        clientID =  sim.simxStart('127.0.0.1', 19990, True, True, 5000, 5)
        if clientID != -1:
            print("Connected to the remote API server")
        else:
            print("Connection not successful")
            sys.exit("Could not connect")


class Marker():

    def __init__(self) -> None:
        e, self.marker = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/Landing_Surface', sim.simx_opmode_oneshot_wait)
        self.coordinates, self.estimated = [], []

    def reset(self, flag = False):
        self.coordinates.clear()
        if flag: # The flag will reset the estimated states too
            self.estimated.clear()

class Drone():

    def __init__(self) -> None:
        e, self.quadcopter = sim.simxGetObjectHandle(clientID, 'Quadcopter', sim.simx_opmode_oneshot_wait)
        e, self.home = sim.simxGetObjectHandle(clientID, '/Cuboid/Home', sim.simx_opmode_oneshot_wait)
        e, self.yaw = sim.simxGetObjectHandle(clientID, 'Yaw', sim.simx_opmode_oneshot_wait)
        e, self.pitch = sim.simxGetObjectHandle(clientID, 'Pitch', sim.simx_opmode_oneshot_wait)
        e, self.roll = sim.simxGetObjectHandle(clientID, 'Roll', sim.simx_opmode_oneshot_wait)
        e, self.cam = sim.simxGetObjectHandle(clientID, 'gimbal_camera', sim.simx_opmode_oneshot_wait)
        e, self.target = sim.simxGetObjectHandle(clientID, '/target', sim.simx_opmode_oneshot_wait)
        e, self.landingSurface = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/Landing_Surface', sim.simx_opmode_oneshot_wait)
        self.state = State.IDLE
        self.start = time.time()
        
    
    def startSim(self, takeOff = True) -> None:
        # Stop the simulation first in case the simulation did not stop properly
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)

        sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

        # Reset Joint Positions

        sim.simxSetJointTargetPosition(clientID, self.yaw, 0, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(clientID, self.pitch, 0, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(clientID, self.roll, 0, sim.simx_opmode_oneshot)


        sim.simxGetVisionSensorImage(clientID, self.cam, 0, sim.simx_opmode_streaming)
        sim.simxGetObjectPosition(clientID, self.quadcopter, self.home, sim.simx_opmode_streaming)
        time.sleep(1)
        if takeOff:
            self.stateTransition(State.TAKEOFF)


    def setState(self, state: State):

        print(f"{self.state.name} state -> {state.name} state \n\n")
        self.state = state
    
    def callStateFunction(self,  marker : Marker = None,  numPlotPoints : int = None, trajectory : np.ndarray = None) -> None: 
        if self.state == State.INITIAL_SEARCH:
            self.initialSearch()
        elif self.state == State.TRACK:
            self.track(marker)
        elif self.state == State.ESTIMATED_SEARCH:
            self.estimatedSearch(marker)
        elif self.state == State.APPROACH:
            self.approach(numPlotPoints, trajectory) # To be implemented
        
    def stateTransition(self, state: State) -> None:
        if self.state == State.IDLE and state == State.TAKEOFF: # IDLE -> TAKEOFF 
            self.takeOff() 
            self.setState(State.TAKEOFF)
        # TakeOff will be called only once

        elif self.state == State.TAKEOFF and state == State.INITIAL_SEARCH: # TAKEOFF -> INITIAL SEARCH
            self.setState(State.INITIAL_SEARCH)


        elif self.state == State.INITIAL_SEARCH and state == State.TRACK: # INITIAL_SEARCH -> TRACK
            self.setState(State.TRACK)


        elif self.state == State.TRACK and state == State.ESTIMATED_SEARCH: # TRACK -> ESTIMATED_SEARCH
            self.setState(State.ESTIMATED_SEARCH)


        elif self.state == State.TRACK and state == State.APPROACH: # TRACK -> APPROACH
            self.setState(State.APPROACH)


        elif self.state == State.ESTIMATED_SEARCH and state == State.INITIAL_SEARCH: # ESTIMATED_SEARCH -> INITIAL_SEARCH
            self.setState(State.INITIAL_SEARCH)


        elif self.state == State.ESTIMATED_SEARCH and state == State.TRACK: # ESTIMATED_SEARCH -> TRACK
            self.setState(State.TRACK)

        elif state == State.LAND:
            self.land()
            self.setState(State.LAND)


    def idle(self):
        print("Quadcopter is now idle")

    def takeOff(self) -> None :

        # Rotate propellers
        sim.simxCallScriptFunction(clientID, '/Quadcopter', sim.sim_scripttype_childscript,  "sysCall_actuation", [1], [], [], '', sim.simx_opmode_blocking)
        
        time.sleep(1)

        # Takeoff
        e, pos = sim.simxGetObjectPosition(clientID, self.quadcopter, self.home, sim.simx_opmode_buffer)
        sim.simxSetObjectPosition(clientID, self.target, self.home, (pos[0], pos[1], 2), sim.simx_opmode_oneshot)
        while pos[2] < 1.9:
            e, pos = sim.simxGetObjectPosition(clientID, self.quadcopter, self.home, sim.simx_opmode_buffer)
        else:
            print("TakeOff successful!")
            self.start = time.time()
    
    def initialSearch(self) -> None :
        e, pos = sim.simxGetObjectPosition(clientID, self.quadcopter, self.home, sim.simx_opmode_buffer)
        if pos[1] < 0.5 and pos[0] < 0.5 and pos[2] < 1.7:
            self.start = time.time()
            sim.simxSetObjectPosition(clientID, self.target, self.home, (pos[0], pos[1], 2), sim.simx_opmode_oneshot)
        else:
            if time.time() - self.start > 15: # Reseting the behaviour
                self.start = time.time()
                sim.simxSetJointTargetPosition(clientID, self.yaw, 0, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetPosition(clientID, self.pitch, 0, sim.simx_opmode_oneshot)
            elif time.time() - self.start > 12:   
                sim.simxSetJointTargetPosition(clientID, self.yaw, -1.570796, sim.simx_opmode_oneshot) # 90 degree rotation. Search towards the front
            elif time.time() - self.start > 9:
                sim.simxSetJointTargetPosition(clientID, self.yaw, 1.570796, sim.simx_opmode_oneshot) # Search towards the front
            elif time.time() - self.start > 6:
                sim.simxSetJointTargetPosition(clientID, self.pitch, searchAngle, sim.simx_opmode_oneshot) # Search towards the right
            elif time.time() - self.start > 3:
                sim.simxSetJointTargetPosition(clientID, self.pitch, -searchAngle, sim.simx_opmode_oneshot) #35 degree rotation. Search towards the left
            else:
                sim.simxSetJointTargetPosition(clientID, self.yaw, 0, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetPosition(clientID, self.pitch, 0, sim.simx_opmode_oneshot)



    def track(self, marker: Marker) -> list:
        global pErrorx, pErrory
        errorx = marker.coordinates[-1][0] - w//2
        errorx = pidx[0]*errorx + pidx[1]*(pErrorx + errorx)//2 + pidx[2]*(pErrorx - errorx)
        _, angx = sim.simxGetJointPosition(clientID, self.pitch, sim.simx_opmode_oneshot)
        
        if angx-math.atan2(errorx, 700) > 0 :
            sim.simxSetJointTargetPosition(clientID, self.pitch, min(angx-math.atan2(errorx, 700), 0.610865), sim.simx_opmode_oneshot) # Giving the camera a maximum rotation of 35 degrees along y axis
        else:
            sim.simxSetJointTargetPosition(clientID, self.pitch, max(angx-math.atan2(errorx, 700), -0.610865), sim.simx_opmode_oneshot) # Giving the camera a maximum rotation of 35 degrees along y axis

        errory = marker.coordinates[-1][1] - h//2
        _, angy = sim.simxGetJointPosition(clientID, self.roll, sim.simx_opmode_oneshot)
        errory = pidy[0]*errory + pidy[1]*(pErrory + errory)//2 + pidy[2]*(pErrory - errory)
        
        if angy-math.atan2(errory, 700) > 0:
            sim.simxSetJointTargetPosition(clientID, self.roll, min(angy-math.atan2(errory, 700), 0.610865), sim.simx_opmode_oneshot) # Giving the camera a maximum rotation of 35 degrees along x axis
        else: 
            sim.simxSetJointTargetPosition(clientID, self.roll, max(angy-math.atan2(errory, 700), -0.610865), sim.simx_opmode_oneshot) # Giving the camera a maximum rotation of 35 degrees along x axis
        pErrorx, pErrory = errorx, errory
    


    def estimatedSearch(self, marker : Marker) -> None:
        global pErrorx, pErrory
        errorx = marker.estimated[-1]   [0] - w//2
        errorx = pidx[0]*errorx + pidx[1]*(pErrorx + errorx)//2 + pidx[2]*(pErrorx - errorx)
        _, angx = sim.simxGetJointPosition(clientID, self.pitch, sim.simx_opmode_oneshot)

        if angx-math.atan2(errorx, 512) > 0 :
            sim.simxSetJointTargetPosition(clientID, self.pitch, min(angx-math.atan2(errorx, 512), 0.610865), sim.simx_opmode_oneshot) # Giving the camera a maximum rotation of 35 degrees along y axis
        else:
            sim.simxSetJointTargetPosition(clientID, self.pitch, max(angx-math.atan2(errorx, 512), -0.610865), sim.simx_opmode_oneshot) # Giving the camera a maximum rotation of 35 degrees along y axis


        errory = marker.estimated[-1]   [1] - h//2
        _, angy = sim.simxGetJointPosition(clientID, self.roll, sim.simx_opmode_oneshot)
        errory = pidy[0]*errory + pidy[1]*(pErrory + errory)//2 + pidy[2]*(pErrory - errory)

        if angy-math.atan2(errory, 512) > 0:
            sim.simxSetJointTargetPosition(clientID, self.roll, min(angy-math.atan2(errory, 512), 0.610865), sim.simx_opmode_oneshot) # Giving the camera a maximum rotation of 35 degrees along x axis
        else: 
            sim.simxSetJointTargetPosition(clientID, self.roll, max(angy-math.atan2(errory, 512), -0.610865), sim.simx_opmode_oneshot) # Giving the camera a maximum rotation of 35 degrees along x axis

        # sim.simxSetJointTargetPosition(clientID, self.roll, angy-math.atan2(errory, 700), sim.simx_opmode_oneshot)
        pErrorx, pErrory = errorx, errory
    
    def approach(self, numPlotPoints, trajectory) -> None:
        for i in range(numPlotPoints):
            sim.simxSetObjectPosition(clientID, self.target, self.home, trajectory[i], sim.simx_opmode_oneshot)
            time.sleep(0.1)

    def land(self) -> None:

        # Turn off propellers
        sim.simxCallScriptFunction(clientID, '/Quadcopter', sim.sim_scripttype_childscript,  "sysCall_actuation", [0], [], [], '', sim.simx_opmode_blocking)
        print("Quadcopter Landing Successful!")







class TrajectoryGenerator():
    def __init__(self, pos, homePosition, vel = [0, 0, 0], acc = [0, 0, 0]) -> None:
        self.initial_pos = np.array(pos)
        self.initial_vel = np.array(vel)
        self.initial_acc = np.array(acc)
        self.gravity = np.array([0, 0, -9.81])
        self.home = homePosition
        self.trajectory = quadtraj.RapidTrajectory(self.initial_pos, self.initial_vel, self.initial_acc, self.gravity)


    def generate(self, drone : Drone, target_pos : list) -> list:

        e, pos = sim.simxGetObjectPosition(clientID, drone.landingSurface, self.home, sim.simx_opmode_oneshot)
        actual_pos = np.array([pos[0], pos[1], pos[2]], dtype = np.float64)
    
        # Update the goal position
        self.trajectory.set_goal_position(np.array(target_pos))
        
        # Set the goal velocity and acceleration to zero (assuming you want the drone to stop at the target)
        self.trajectory.set_goal_velocity(np.array([0, 0, 0]))
        self.trajectory.set_goal_acceleration(np.array([0, 0, 0]))
        
        # Generate the self.trajectory with an example timeToGo
        time_to_go = 5  # This should be adjusted based on your requirements
        self.trajectory.generate(time_to_go)
        
        # Example usage: get the position, velocity, and acceleration at time t
        # Example time
        
        numPlotPoints = 100
        pos = np.zeros((numPlotPoints, 3), dtype=np.float64)
        time = np.linspace(0, time_to_go, numPlotPoints) # generates 100 evenly spaced points between 0 and 2

        
        for i in range(numPlotPoints):
            t = time[i]
            pos[i, :] = self.trajectory.get_position(t)
        # print(pos)
        return [pos, numPlotPoints] 




class ObjectDetection():

    def __init__(self, drone : Drone) -> None:
        
        e, pos = sim.simxGetObjectPosition(clientID, drone.quadcopter, drone.home, sim.simx_opmode_streaming)
        self.searchTime, self.trackTime = time.time(), time.time()
        self.traj = TrajectoryGenerator(pos, homePosition=drone.home) # Initial Coordinates passed to trajectory generator

        self.target_ref_pos = None

    def detection(self, drone: Drone, marker: Marker):
        
        e, pos = sim.simxGetObjectPosition(clientID, drone.quadcopter, drone.home, sim.simx_opmode_buffer)
        e, markerPos = sim.simxGetObjectPosition(clientID, marker.marker, drone.home, sim.simx_opmode_buffer)
        e, resolution, image = sim.simxGetVisionSensorImage(clientID, drone.cam, 0, sim.simx_opmode_buffer)
        

        if e == sim.simx_return_ok:
        
            image = 255+np.array(image)
            image = image.astype(np.uint8)
            image = np.reshape(image, (resolution[1], resolution[0], 3))
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)


            noisy_image = utils.addNoise(image) # Adds gaussian noise


            if(pos[2] > 1.00) : # Blob detection in case the marker is at a height greater than 1m from the ground.  
                
                imgContours, contours = utils.detectBlobEdges(noisy_image, minArea = 500)
                if len(contours): # Landing surface has been detected
                    self.searchTime = time.time()
                    # Gets the central coordinate of the largest contour. contour array is sorted in descending order by default.

                    cx, cy = contours[0]['center']
                    cv2.circle(imgContours, (cx, cy), 3, (0, 255, 0), cv2.FILLED)

                    marker.coordinates.append([cx, cy])
                    marker.estimated.append(utils.stateEstimation(noisy_image, marker.coordinates, cx, cy))

                    drone.stateTransition(State.TRACK) if drone.state != State.TRACK else drone.callStateFunction(marker) # Attempts to track the marker


                    vals = sim.simxCallScriptFunction(clientID, '/Quadcopter/gimbal_camera', sim.sim_scripttype_childscript,  "sysCall_sensing", [marker.estimated[-1][0], marker.estimated[-1][1]], [], [], '', sim.simx_opmode_blocking) 
                    # obtainedTrajectory, numPlotPoints = self.traj.generate(drone, vals[2])

                    e, drone_pos = sim.simxGetObjectPosition(clientID, drone.quadcopter, drone.home, sim.simx_opmode_oneshot)
                    
                    if self.target_ref_pos is None:    
                        self.target_ref_pos = drone_pos

                    self.target_ref_pos = [0.1*vals[2][0]   + 0.9*self.target_ref_pos[0],
                                           0.1*vals[2][1]   + 0.9*self.target_ref_pos[1],
                                           0.01*vals[2][2] + 0.99*self.target_ref_pos[2] ]

                    sim.simxSetObjectPosition(clientID, drone.target, drone.home, self.target_ref_pos, sim.simx_opmode_oneshot)
                    
                     

                        

                    # drone.stateTransition(State.APPROACH) if drone.state != State.APPROACH else drone.callStateFunction( trajectory=obtainedTrajectory, numPlotPoints=numPlotPoints) # Attempts to track the marker
                    

                elif len(marker.estimated) == 0 or time.time() - self.searchTime > 10: # Either marker has not been detected or 10 seconds have passed since the last detection 
                    drone.stateTransition(State.INITIAL_SEARCH) if drone.state != State.INITIAL_SEARCH else drone.callStateFunction()
                    marker.reset(True)

                elif len(marker.estimated):
                    drone.stateTransition(State.ESTIMATED_SEARCH) if drone.state != State.ESTIMATED_SEARCH else drone.callStateFunction(marker, pErrorx, pErrory)
                    marker.reset()

                cv2.imshow("ImgContours", imgContours)
                cv2.waitKey(1)

            else:
                # cv2.destroyWindow('imgContours')
                markerCorners, frame_markers = utils.detectMarker(noisy_image, dictionary)  # detected markers
                cx, cy = 0.0, 0.0
                if len(markerCorners):
                    self.searchTime = time.time()
                    for corner in markerCorners:
                        cx += (corner[0][0][0] + corner[0][1][0] + corner[0][2][0] + corner[0][3][0]) / 4
                        cy += (corner[0][0][1] + corner[0][1][1] + corner[0][2][1] + corner[0][3][1]) / 4
                    cv2.circle(frame_markers, (int(cx), int(cy)), 3, (0, 255, 0), cv2.FILLED)
                    marker.estimated.append(utils.stateEstimation(frame_markers, marker.coordinates, cx, cy))

                    vals = sim.simxCallScriptFunction(clientID, '/Quadcopter/gimbal_camera', sim.sim_scripttype_childscript,  "sysCall_sensing", [marker.estimated[-1][0], marker.estimated[-1][1]], [], [], '', sim.simx_opmode_blocking) 

                    drone.stateTransition(State.TRACK) if drone.state != State.TRACK else drone.callStateFunction(marker) # Attempts to track the marker


                    e, drone_pos = sim.simxGetObjectPosition(clientID, drone.quadcopter, drone.home, sim.simx_opmode_oneshot)
                    if self.target_ref_pos is None:    
                        self.target_ref_pos = drone_pos


                    self.target_ref_pos = [0.1*vals[2][0] + 0.9*self.target_ref_pos[0],
                                      0.1*vals[2][1] + 0.9*self.target_ref_pos[1],
                                      0.01*vals[2][2] + 0.99*self.target_ref_pos[2] ]
                   

                    sim.simxSetObjectPosition(clientID, drone.target, drone.home, self.target_ref_pos, sim.simx_opmode_oneshot)

                    if pos[2] < 0.4:
                        drone.stateTransition(State.LAND) if drone.state != State.LAND else  sys.exit()


                elif len(marker.estimated) == 0 or time.time() - self.searchTime > 10: # Either marker has not been detected or 10 seconds have passed since the last detection 
                    drone.stateTransition(State.INITIAL_SEARCH) if drone.state != State.INITIAL_SEARCH else drone.callStateFunction()
                    marker.reset(True)

                elif len(marker.estimated):
                    drone.stateTransition(State.ESTIMATED_SEARCH) if drone.state != State.ESTIMATED_SEARCH else drone.callStateFunction(marker, pErrorx, pErrory)
                    marker.estimated.append(utils.stateEstimation(noisy_image, marker.coordinates, marker.estimated[-1][0], marker.estimated[-1][1]))
                    marker.reset()

                cv2.imshow("Frame_Markers", frame_markers)
                cv2.waitKey(1)


                    

