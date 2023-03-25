import cv2
import imutils
import numpy as np

'''
for visualization testing purposes, you can include imshow from this code:
https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/blob/main/utils.py
'''

class AruCoDetector():
    def __init__(self, resize_width):
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.resize_width = resize_width

    def detect(self, frame):
        image = imutils.resize(image, width=self.resize_width)

        corners, _, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        
        xy = np.zeros(2)

        tl, _, br, _ = corners[0].reshape((4, 2)).astype(int)

        xy = ((tl + br) / 2).astype(int)

        return xy[0], xy[1]
    