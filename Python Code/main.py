import sim
import sys
import signal
import cv2
import numpy as np
import utils
import time
from cv2 import aruco

coordinates, estimatedCoordinates, pidx, pidy, pErrorx, pErrory, searchAngle =  [], [], [0.1, 0.1, 0.5], [0.1, 0.1, 0.5], 0, 0, 0.610865
detected = False    


# aruco marker dictionary
dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19990, True, True, 5000, 5)
print(clientID)
if clientID != -1:
    print("Connected to the remote API server")
else:
    print("Connection not successful")
    sys.exit("Could not connect")
# Stop the simulation first in case the simulation did not stop properly
sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)

sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

def signal_handler(signal, frame):
    sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Obtaining the required handles

e, camHandle = sim.simxGetObjectHandle(clientID, 'gimbal_camera', sim.simx_opmode_oneshot_wait)
e, viewHandle = sim.simxGetObjectHandle(clientID, 'viewer', sim.simx_opmode_oneshot_wait)
e, quadcopterHandle = sim.simxGetObjectHandle(clientID, 'Quadcopter', sim.simx_opmode_oneshot_wait)
e, landingSurface = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/Landing_Surface', sim.simx_opmode_oneshot_wait)
e, targetHandle = sim.simxGetObjectHandle(clientID, '/target', sim.simx_opmode_oneshot_wait)

# Joints

e, yawHandle = sim.simxGetObjectHandle(clientID, 'Yaw', sim.simx_opmode_oneshot_wait)
e, pitchHandle = sim.simxGetObjectHandle(clientID, 'Pitch', sim.simx_opmode_oneshot_wait)
e, rollHandle = sim.simxGetObjectHandle(clientID, 'Roll', sim.simx_opmode_oneshot_wait)

# Reset Joint Positions

sim.simxSetJointPosition(clientID, yawHandle, 0, sim.simx_opmode_oneshot)
sim.simxSetJointPosition(clientID, pitchHandle, 0, sim.simx_opmode_oneshot)
sim.simxSetJointPosition(clientID, rollHandle, 0, sim.simx_opmode_oneshot)


sim.simxGetVisionSensorImage(clientID, camHandle, 0, sim.simx_opmode_streaming)
sim.simxGetObjectPosition(clientID, quadcopterHandle, -1, sim.simx_opmode_streaming)




# TAKEOFF!
time.sleep(1) # wait for 1 second!

start = utils.takeOff(clientID, 1010077, quadcopterHandle, targetHandle)


while True:
    e, resolution, image = sim.simxGetVisionSensorImage(clientID, camHandle, 0, sim.simx_opmode_buffer)
    e, pos = sim.simxGetObjectPosition(clientID, quadcopterHandle, -1, sim.simx_opmode_buffer) # pos is a list of [x, y, z]
    

    if e == sim.simx_return_ok:
        
        image = 255+np.array(image)
        image = image.astype(np.uint8)
        image = np.reshape(image, (resolution[1], resolution[0], 3))
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if(pos[2] > 1.00) : # Blob detection in case the marker is at a height greater than 1m from the ground.      
            imgContours, contours = utils.detectBlob(image)

            if len(contours): # Landing surface has been detected
                    # Gets the central coordinate of the largest contour. contour array is sorted in descending order by default.
                cx, cy = contours[0]['center']
                coordinates.append([cx, cy])
                
                cv2.circle(imgContours, (cx, cy), 3, (0, 255, 0), cv2.FILLED)
                estimatedCoordinates.append(utils.stateEstimation(imgContours, coordinates, cx, cy))
                e, landingPos = sim.simxGetObjectPosition(clientID, landingSurface, -1 ,sim.simx_opmode_buffer)

                # if(abs(estimatedCoordinates[-1][0] - cx ) < 5 and abs(estimatedCoordinates[-1][1] - cy) < 5):
                #     utils.trajectoryGen(estimatedCoordinates[-1][0], estimatedCoordinates[-1][1], landingPos)
                # print("Here")
                # utils.trajectoryGen(estimatedCoordinates[-1][0], estimatedCoordinates[-1][1], landingPos)
                
                vals = sim.simxCallScriptFunction(clientID, '/Quadcopter/gimbal_camera', sim.sim_scripttype_childscript,  "sysCall_sensing", [cx, cy], [], [], '', sim.simx_opmode_blocking)

                detected = True
                pErrorx, pErrory = utils.trackLandingMarker(resolution[0], resolution[1], cx, cy, pErrorx, pErrory, pidx, pidy, clientID, yawHandle, pitchHandle, rollHandle)
                #sim.simxSetJointTargetVelocity
            elif not detected: #This code will make the camera look in each direction for 3 seconds.  
                if time.time() - start > 15: # Reseting the behaviour
                    start = time.time()
                    sim.simxSetJointPosition(clientID, yawHandle, 0, sim.simx_opmode_oneshot)
                    sim.simxSetJointPosition(clientID, pitchHandle, 0, sim.simx_opmode_oneshot)
                elif time.time() - start > 12:   
                    sim.simxSetJointPosition(clientID, yawHandle, -1.570796, sim.simx_opmode_oneshot) # 90 degree rotation. Search towards the front
                elif time.time() - start > 9:
                    sim.simxSetJointPosition(clientID, yawHandle, 1.570796, sim.simx_opmode_oneshot) # Search towards the front
                elif time.time() - start > 6:
                    sim.simxSetJointPosition(clientID, pitchHandle, searchAngle, sim.simx_opmode_oneshot) # Search towards the right
                elif time.time() - start > 3:
                    # utils.searchMarker(clientID, pitchHandle, searchAngle, 0.2, sim.simx_opmode_oneshot)
                    sim.simxSetJointPosition(clientID, pitchHandle, -searchAngle, sim.simx_opmode_oneshot) #35 degree rotation. Search towards the left
            else:
                estimatedCoordinates.append(utils.stateEstimation(imgContours, estimatedCoordinates, estimatedCoordinates[-1][0], estimatedCoordinates[-1][1]))
                
                
            cv2.imshow("imgContours", imgContours)
            


        else: #switch to aruco marker detection
            # image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            markerCorners, frame_markers = utils.detectMarker(image, dictionary)  # detected markers
            
            cx, cy = 0.0, 0.0
            sim.simxSetVisionSensorImage(clientID, viewHandle, frame_markers.flatten(), 0, sim.simx_opmode_oneshot)
            for corner in markerCorners:
                cx += (corner[0][0][0] + corner[0][1][0] + corner[0][2][0] + corner[0][3][0]) / 4
                cy += (corner[0][0][1] + corner[0][1][1] + corner[0][2][1] + corner[0][3][1]) / 4
            if len(markerCorners):
                cv2.circle(frame_markers, (int(cx), int(cy)), 3, (0, 255, 0), cv2.FILLED)
                utils.stateEstimation(frame_markers, coordinates, cx, cy)
            # cv2.imshow("Frame_Markers", frame_markers) 


    else:
        print("Empty Return")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
        break
