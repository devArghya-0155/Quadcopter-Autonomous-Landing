import cv2
import numpy
from cv2 import aruco

dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
cap = cv2.VideoCapture(0)

while(True):
    #Capture frame-by-frame
    ret, frame = cap.read()

    # Convert img to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, parameters)
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
    frame_markers = aruco.drawDetectedMarkers (frame.copy(), corners, ids)

    cv2.imshow('Aruco Markers', frame_markers)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

