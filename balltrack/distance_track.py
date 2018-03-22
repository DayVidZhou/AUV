# import the necessary packages
from collections import deque
import numpy as np
import argparse
import imutils
import cv2

# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
font = cv2.FONT_HERSHEY_SIMPLEX

camera = cv2.VideoCapture(0)

while True:
    (grabbed, frame) = camera.read()

    frame = imutils.resize(frame, width=1000)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
    # find the largest contour in the mask, then use
    # it to compute the minimum enclosing circle and centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        P = 192
        W = 42.7
        D = 150
        F=P*D/W
        distance = (W*F)/(int(radius)*20) 
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            cv2.putText(frame,str(distance), center, font, 1, (255,255,255), 
                2, cv2.LINE_AA)
    # show the frame to our screen
    cv2.imshow("Frame", frame)
    cv2.imshow("mask", mask)
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
