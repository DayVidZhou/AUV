import numpy as np
import argparse
import imutils
import cv2
import time

ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help = "video name")
args=vars(ap.parse_args())

#Global vars
global frame
global pic
global drawn
curArea = ""
drawn = False
pic = 7495

#HSV boundaries of the balls
greenLower = (29, 150, 106)
greenUpper = (50, 255, 255)
pinkLower = (160, 109, 20)                                                      
pinkUpper = (200, 255, 255)
redLower = (3, 150, 80)
redUpper = (15, 255, 255)
#[[[  5 248 116]]]
#[[[  8 211 122]]]
#[[[ 10 175  92]]]
#[[[  7 228 162]]]
font = cv2.FONT_HERSHEY_SIMPLEX

camera = cv2.VideoCapture(args["video"])

(grabbed, frame) = camera.read()

def maskcont():
    global image
    global maxg
    global maxp
    global maxr
    global gMask
    global pMask
    global rMask
    maxr = np.zeros((6,1,2))
    maxg = np.zeros((6,1,2))
    maxp = np.zeros((6,1,2))
    image = frame
    image = imutils.resize(image, width=700)
                                                                                  
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)                                
                                                                                  
    #set masks                                                                  
    gMask = cv2.inRange(hsv, greenLower, greenUpper)                            
    gMask = cv2.erode(gMask, None, iterations=2)                                
    gMask = cv2.dilate(gMask, None, iterations=2)                               
    pMask = cv2.inRange(hsv, pinkLower, pinkUpper)                              
    pMask = cv2.erode(pMask, None, iterations=2)                                
    pMask = cv2.dilate(pMask, None, iterations=2)                               
    rMask = cv2.inRange(hsv, redLower, redUpper)                            
    rMask = cv2.erode(rMask, None, iterations=2)                                
    rMask = cv2.dilate(rMask, None, iterations=2)                               
    #find contours                                                              
    gcnts = cv2.findContours(gMask.copy(), cv2.RETR_EXTERNAL,                   
          cv2.CHAIN_APPROX_SIMPLE)[-2]                                        
    pcnts = cv2.findContours(pMask.copy(), cv2.RETR_EXTERNAL,                   
          cv2.CHAIN_APPROX_SIMPLE)[-2]                                        
    rcnts = cv2.findContours(rMask.copy(), cv2.RETR_EXTERNAL,                   
          cv2.CHAIN_APPROX_SIMPLE)[-2]                                        
    if gcnts:
        maxg = max(gcnts, key=cv2.contourArea)
        cv2.putText(gMask, str(cv2.contourArea(maxg)), (20, 50), font, 1, (255), 2,
            cv2.LINE_AA)
    if pcnts:  
        maxp = max(pcnts, key=cv2.contourArea)
        cv2.putText(pMask, str(cv2.contourArea(maxp)), (20, 50), font, 1, (255), 2,
            cv2.LINE_AA)
    if rcnts:
        maxr = max(rcnts, key=cv2.contourArea)
        cv2.putText(rMask, str(cv2.contourArea(maxr)), (20, 50), font, 1, (255), 2,
            cv2.LINE_AA)

def getArea():
    global curArea
    if not curArea:
        curArea = "1"
    if (curArea == "1" and maxp.all()):
        if cv2.contourArea(maxp) > 1000:
            curArea = "2"
            return 0
    if (maxg.all() and curArea == "2"):
        if cv2.contourArea(maxg) > 250:
            curArea = "3"
            return 0
    cv2.putText(image, curArea, (20, 50), font, 2, (255, 255, 255), 2, 
            cv2.LINE_AA)

while True:
    if not drawn:
        maskcont()
        getArea()
        #show frams
        cv2.imshow("Image", image)
        cv2.imshow("Gmask", gMask)
        cv2.imshow("Pmask", pMask)
        cv2.imshow("Rmask", rMask)
        drawn = True

    #if q pressed quit out
    k = cv2.waitKey(1) & 0xFF
    if k == ord("n"):
        (grabbed, frame) = camera.read()
        if not grabbed:
            break
        drawn = False

cv2.destroyAllWindows()
