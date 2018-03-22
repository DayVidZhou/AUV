import picamera
import time
import cv2
import argparse

ap=argparse.ArgumentParser()
ap.add_argument("-n", "--name", required = True, help = "name of video")
ap.add_argument("-l", "--length", required = True, help = "length")
args = vars(ap.parse_args())
vidname = args["name"] + ".h264"

camera = picamera.PiCamera()
camera.framerate = 5

camera.start_recording(vidname)

time.sleep(float(args["length"]))

camera.stop_recording()

