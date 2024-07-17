
import math
import time
import sim
import cv2
from cv2 import aruco
import numpy as np
from skimage.util import random_noise

# My files
import kalmanFilter
import colorDetection
import trajectoryGenerator

hsvVals = {'hmin': 0, 'smin': 102, 'vmin': 107, 'hmax': 0, 'smax': 255, 'vmax': 255}  # hsvVals for our mask
threshold1, threshold2 = 219, 255 # threshold values for canny edge detector

coordinates, cameraCoordinates = [], [
    # Here, coordinates are with respect to camera frame, not drone frame.
    0.0, 0.0, -0.0289]
cameraOpticalCenter = [256//2, 256//2]

filtered_state_mean = np.zeros((4, 1))
filtered_state_covariance = np.eye(4)



def addNoise(image):
    noisy_image = random_noise(image, mode = "gaussian", var = 0.02**2)
    noisy_image = (255*noisy_image).astype(np.uint8)
    return noisy_image



def drawContours(image, mask, minArea = 100, contourColor = (255, 0, 0), contourTextColor = (0, 0, 255)):
    imgContours = image[:]
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contoursFound = []

 
    for cntr in contours:
        area = cv2.contourArea(cntr)
        if minArea < area :
            peri = cv2.arcLength(cntr, True)
            approx = cv2.approxPolyDP(cntr, 0.02 * peri, True)
            cv2.drawContours(imgContours, cntr, -1, contourColor, 3)
            x, y, w, h = cv2.boundingRect(approx)
            cv2.putText(imgContours, str(len(approx)), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, contourTextColor, 2)
            cx, cy = x + (w // 2), y + (h // 2)
            cv2.rectangle(imgContours, (x, y), (x + w, y + h), contourColor, 2)
            cv2.circle(imgContours, (x + (w // 2), y + (h // 2)), 3, contourColor, cv2.FILLED)
            contoursFound.append({"cntr": cntr, "area": area, "bbox": [x, y, w, h], "center": [cx, cy]})

    contoursFound = sorted(contoursFound, key=lambda x: x["area"], reverse=True ) # sorts the list in descending order based on area

    return imgContours, contoursFound

def detectBlobEdges(image, flag = False, minArea = 100):
    if not flag:
        
        imgBlur = cv2.GaussianBlur(image, (7,7), 1)
        imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
        imgCanny = cv2.Canny(imgGray, threshold1, threshold2) # Canny edge detection

        imgDl = cv2.dilate(imgCanny, np.ones((5, 5)), iterations = 1)
        imgContours, contours = drawContours(image, imgDl, minArea)
        return imgContours, contours
    else:
        
        imgDl = colorDetection.detectBlobEdges(image, 500)
        imgContours, contours = drawContours(image, imgDl, 500)
        return imgContours, contours

        


def detectBlobHSV(image, flag = False):
    if not flag:
        hsvImage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsvImage, np.array([hsvVals['hmin'], hsvVals['smin'], hsvVals['vmin']], dtype = np.uint8), np.array([hsvVals['hmax'], hsvVals['smax'], hsvVals['vmax']], dtype = np.uint8))
        result = cv2.bitwise_and(image, image, mask)
        imgContours, contours = drawContours(image, mask, 200)    
        return imgContours, contours
    else:
        mask, imgColor = colorDetection.detectBlobHSV(image)
        imgContours, contours = drawContours(image, mask)
        return imgContours, contours
    
def detectMarker(img, dictionary):

    # Convert img to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, parameters)
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    return [corners, aruco.drawDetectedMarkers(img.copy(), corners, ids)]


def trackLandingMarker(w, h, cx, cy, pErrorx, pErrory, pidx, pidy, clientID, yawHandle, pitchHandle, rollHandle):
    errorx = cx - w//2
    errorx = pidx[0]*errorx + pidx[1]*(pErrorx + errorx)//2 + pidx[2]*(pErrorx - errorx)
    _, angx = sim.simxGetJointPosition(clientID, pitchHandle, sim.simx_opmode_oneshot)

    if angx-math.atan2(errorx, 700) > 0 :
        sim.simxSetJointPosition(clientID, pitchHandle, min(angx-math.atan2(errorx, 700), 0.610865), sim.simx_opmode_oneshot) # Giving the camera a maximum rotation of 35 degrees along y axis
    else:
        sim.simxSetJointPosition(clientID, pitchHandle, max(angx-math.atan2(errorx, 700), -0.610865), sim.simx_opmode_oneshot) # Giving the camera a maximum rotation of 35 degrees along y axis
        

    errory = cy - h//2
    _, angy = sim.simxGetJointPosition(clientID, rollHandle, sim.simx_opmode_oneshot)
    errory = pidy[0]*errory + pidy[1]*(pErrory + errory)//2 + pidy[2]*(pErrory - errory)

    if angy-math.atan2(errory, 700) > 0:
        sim.simxSetJointPosition(clientID, rollHandle, min(angy-math.atan2(errory, 700), 0.610865), sim.simx_opmode_oneshot) # Giving the camera a maximum rotation of 35 degrees along x axis
    else: 
        sim.simxSetJointPosition(clientID, rollHandle, max(angy-math.atan2(errory, 700), -0.610865), sim.simx_opmode_oneshot) # Giving the camera a maximum rotation of 35 degrees along x axis
    return [errorx, errory]


def stateEstimation(img, coordinates, cx, cy):
    if(len(coordinates) > 50):
        coordinates.pop(0)
    coordinates.append([cx, cy])
    val1, val2 = kalmanFilter.predict(np.array(coordinates))
    predictX, predictY = val1[0], val1[2]
    cv2.circle(img, (predictX, predictY), 3, (0, 0, 255), cv2.FILLED)
    return [predictX, predictY]