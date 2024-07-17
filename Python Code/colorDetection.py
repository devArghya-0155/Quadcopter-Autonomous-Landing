import cv2
import numpy as np


def empty(a):
    pass

# cv2.namedWindow("HSV")
# cv2.resizeWindow("HSV",640,240)
# cv2.createTrackbar("HUE Min","HSV",0,179,empty)
# cv2.createTrackbar("HUE Max","HSV",179,179,empty)
# cv2.createTrackbar("SAT Min","HSV",0,255,empty)
# cv2.createTrackbar("SAT Max","HSV",255,255,empty)
# cv2.createTrackbar("VALUE Min","HSV",0,255,empty)
# cv2.createTrackbar("VALUE Max","HSV",255,255,empty)

cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters", 640, 120)
cv2.createTrackbar("Threshold1", "Parameters", 150, 255, empty)
cv2.createTrackbar("Threshold2", "Parameters", 150, 255, empty)


def detectBlobEdges(img, minArea = 100):
    contourColor, contourTextColor = (255, 0, 0), (0, 0, 255)
    imgBlur = cv2.GaussianBlur(img, (7,7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)

    threshold1, threshold2 = cv2.getTrackbarPos("Threshold1", "Parameters"), cv2.getTrackbarPos("Threshold2", "Parameters")
    imgCanny = cv2.Canny(imgGray, threshold1, threshold2) # Canny edge detection

    imgDl = cv2.dilate(imgCanny, np.ones((3, 3)), iterations = 2)
    cv2.imshow("dilated image", imgDl)

    return imgDl


def detectBlobHSV(img, hsvVals = False):
    hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower, upper = 0, 0
    if hsvVals == False: # Create Trackbar if no hsvVals have been provided

        hmin, hmax = cv2.getTrackbarPos("HUE Min", "HSV"), cv2.getTrackbarPos("HUE Max", "HSV")
        smin, smax = cv2.getTrackbarPos("SAT Min", "HSV"), cv2.getTrackbarPos("SAT Max", "HSV")
        vmin, vmax = cv2.getTrackbarPos("VALUE Min", "HSV"), cv2.getTrackbarPos("VALUE Max", "HSV")

        lower, upper = np.array([hmin, smin, vmin], dtype = np.uint8), np.array([hmax, smax, vmax], dtype = np.uint8)
    else: # use provided hsvVals
        lower, upper = np.array([hsvVals['hmin'], hsvVals['smin'], hsvVals['vmin']], dtype = np.uint8), np.array([hsvVals['hmax'], hsvVals['smax'], hsvVals['vmax']], dtype = np.uint8)
        


    mask = cv2.inRange(hsvImg, lower, upper) #mask of hsvVals in range of given number
    result = cv2.bitwise_and(img, img, mask = mask)
    stacked = np.hstack([img, result])
    cv2.imshow("mask", mask)
    cv2.imshow("Stacked Images", stacked)
    return mask, result
