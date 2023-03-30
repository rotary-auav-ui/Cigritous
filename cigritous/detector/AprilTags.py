import cv2
import imutils
import numpy as np
import pupil_apriltags as apriltag

class AprilTags():
    def __init__(self, resize_width):
        self.detector = apriltag.Detector(families="tag36h11")

        self.resize_width = resize_width

    def detect(self, frame):
        frame = imutils.resize(frame, width=self.resize_width)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = self.detector.detect(frame)

        xy = np.zeros(2)

        for r in results:
            '''
            you can extract the detected corners 
            too by accessing 'r.corners'
            '''
            xy = r.center.astype(int)

        return xy[0], xy[1]
    
