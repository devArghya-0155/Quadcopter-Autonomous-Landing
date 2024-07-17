import cv2
from djitellopy import Tello

def intializeTello():
    # CONNECT TO TELLO
    myDrone = Tello()
    myDrone.connect()
    myDrone.for_back_velocity = 0
    myDrone.left_right_velocity = 0
    myDrone.up_down_velocity = 0
    myDrone.yaw_velocity = 0
    myDrone.speed = 0
    print(myDrone.get_battery())
    myDrone.streamoff()
    myDrone.streamon()
    return myDrone

def telloGetFrame(myDrone, scale = 0.2):
    myFrame = (myDrone.get_frame_read()).frame
    # myFrame = myFrame.frame
    return cv2.resize(myFrame, (int(myFrame.shape[0] * scale), int(myFrame.shape[1] * scale)))

def findFace(img):
    faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml") # Mainly responsible for face detection magic. Must replace it with Aruco Marker detection logic
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGray, 1.1, 4)
    myFacesListC , myFaceListArea = [], [] #Stores the center coordinates and area of faces. Logic used in this code simply finds the largest face area and tracks that. The largest face will be the one that is closest to the screen.
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cx = x + w//2
        cy = y + h//2
        area = w*h
        myFacesListC.append([cx,cy])
        myFaceListArea.append(area)
    if len(myFaceListArea) != 0:
        i = myFaceListArea.index(max(myFaceListArea))
        # index of closest face
        return img,[myFacesListC[i],myFaceListArea[i]]
    else:
        return img, [[0,0],0]


myDrone = intializeTello()

while True:
    ## STEP 1
    img = telloGetFrame(myDrone)
    # DISPLAY IMAGE
    cv2.imshow("MyResult", img)
    img, c = findFace(img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        myDrone.land()
        break