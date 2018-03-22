# import the necessary packages
import numpy as np
import argparse
import cv2
import time

# construct argument parser and parse the argument
#ap = argparse.ArgumentParser()
#ap.add_argument("-i", "--image", required = True, help = "Path to image")
#args = vars(ap.parse_args())

font = cv2.FONT_HERSHEY_SIMPLEX
posx = 0
posy = 0

def mouseclick(event, x, y, flag, param):
    global posx
    global posy
    if event == cv2.EVENT_LBUTTONDOWN:
            posx = x
            posy = y
while True:
    #load the image, clone it for output and convert to grayscale
    image = cv2.imread('/Users/maclovin/Pictures/green.jpg')
    #output = image.copy()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 25)
    #Detect circles
    
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 200)

    #make sure some circle was found
    if circles is not None:
        # get x, y and radius
        circles = np.round(circles[0, :]).astype("int")

        #loop over x, y and r
        for (x, y, r) in circles:
            #draw circle in output
            cv2.circle(image, (x, y), r, (0,255,0), 4)
        
        #show output image
        #cv2.imshow("Image", image)
      #  cv2.imshow("Output", output)
    cv2.setMouseCallback("Image", mouseclick, 0) 
    key = cv2.waitKey(1) & 0xFF

    color = np.uint8([[[46,168,144]]])
    color2 = np.uint8([[[35,130,93]]])
    color3 = np.uint8([[[9, 175, 159]]])
    color4 = np.uint8([[[0, 112, 8]]])
    hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    hsv_color2 = cv2.cvtColor(color2, cv2.COLOR_BGR2HSV)
    hsv_color3 = cv2.cvtColor(color3, cv2.COLOR_BGR2HSV)
    hsv_color4 = cv2.cvtColor(color4, cv2.COLOR_BGR2HSV)
    print(hsv_color)
    print(hsv_color2)
    print(hsv_color3)
    print(hsv_color4)
    break

    if key == ord("q"):
        break
    if posx and posy:
        text = "x: " + str(posx) + " y: " + str(posy)
        cv2.circle(image, (posx, posy), 5, (0, 0, 255), 4)
        cv2.putText(image, text, (20, 20), font, 1, (255, 255, 255), 1,
                cv2.LINE_AA)
        color = np.uint8([[[45, 51, 151]]])
        color2 = np.uint8([[[69, 81, 119]]])
        hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        hsv_color2 = cv2.cvtColor(color2, cv2.COLOR_BGR2HSV)
        #hsv_text = "HSV: " + str(hsv_color[0,0,0]) + ", " + str(hsv_color[0,0,1]) + ", " + str(hsv_color[0,0,2])
        #print (hsv_color)
        hsv_text = "HSV: " + str(hsv_color)
        cv2.putText(image, hsv_text, (20, 50), font, 1, (255, 255, 255), 1,
                cv2.LINE_AA)
        cv2.putText(image, str(hsv_color2), (20, 100), font, 1, (255, 255,255), 1, 
                cv2.LINE_AA)
    cv2.imshow("Image", image)
cv2.destroyAllWindows()
