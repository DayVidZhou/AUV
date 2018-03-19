import picamera
import time
import cv2
import argparse

ap=argparse.ArgumentParser()
ap.add_argument("-n", "--name", required = True, help = "name of video")
args = vars(ap.parse_args())
vidname = args["name"] + ".h264"


camera = picamera.PiCamera()

camera.start_recording(vidname)
while True:
    c = cv2.waitKey(1) & 0xFF
    if c:
        break

camera.stop_recording()

