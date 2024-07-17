import sim
import sys
import numpy as np
import cv2
from pynput.keyboard import Key, Listener


# def on_press(key):
#     if key == Key.enter:
#         return False #Stop process when enter has been pressed.


# # Collect events until released
# with Listener(
#         on_press=on_press) as listener:
#     listener.join()


def nothing(pos):
    pass


def obtainBlob(frame):

    cv2.namedWindow('Thresholds')
    cv2.createTrackbar('LH', 'Thresholds', 0, 255, nothing)
    cv2.createTrackbar('LS', 'Thresholds', 0, 255, nothing)
    cv2.createTrackbar('LV', 'Thresholds', 0, 255, nothing)
    cv2.createTrackbar('UH', 'Thresholds', 255, 255, nothing)
    cv2.createTrackbar('US', 'Thresholds', 255, 255, nothing)
    cv2.createTrackbar('UV', 'Thresholds', 255, 255, nothing)

    while True:
        color_lower = np.empty((3, 3), np.uint8)
        color_upper = np.empty((3, 3), np.uint8)
        if len(frame):

            newImg = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            lh, ls, lv = cv2.getTrackbarPos('LH', 'Thresholds'), cv2.getTrackbarPos(
                'LS', 'Thresholds'), cv2.getTrackbarPos('LV', 'Thresholds')
            uh, us, uv = cv2.getTrackbarPos('UH', 'Thresholds'), cv2.getTrackbarPos(
                'US', 'Thresholds'), cv2.getTrackbarPos('UV', 'Thresholds')

            # defining the Range of color
            color_lower = np.array([lh, ls, lv], np.uint8)
            color_upper = np.array([uh, us, uv], np.uint8)

            # obtaining mask
            mask = cv2.inRange(newImg, color_lower, color_upper)
            kernel = np.ones((5, 5), "uint8")

            detected = cv2.dilate(mask, kernel)
            cv2.imshow("Color", detected)
            cv2.imshow("Original Image", frame)

        if cv2.waitKey(1) & 0xFF == ord('d'):
            # numpy.savetxt(fname, X, fmt='%.18e', delimiter=' ', newline='\n', header='', footer='', comments='# ', encoding=None)
            np.savetxt("Blob_lower.txt", color_lower)
            np.savetxt("Blob_upper.txt", color_upper)
            # print(color_upper, color_lower)
            break
