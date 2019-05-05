import cv2
import numpy as np
import config
import time
# capturing video through webcam

# cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,640);
# cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 480);


def traffic_light(img1):
    # img=img1[10:400,200:400] #for rightmost ROI
    # img = img1[30:2000, 500:700]  # for leftmost ROI

    img = cv2.resize(img1,( 600, 600))
    # converting frame(img i.e BGR) to HSV (hue-saturation-value)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # definig the range of red color
    red_lower = np.array([156, 43, 46], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)

    # defining the Range of green color
    green_lower = np.array([35, 43, 46], np.uint8)
    green_upper = np.array([77, 255, 255], np.uint8)

    # finding the range of red,green color in the image
    red = cv2.inRange(hsv, red_lower, red_upper)
    green = cv2.inRange(hsv, green_lower, green_upper)

    # Morphological transformation, Dilation
    kernal = np.ones((5, 5), "uint8")

    red = cv2.dilate(red, kernal)
    res = cv2.bitwise_and(img, img, mask=red)

    green = cv2.dilate(green, kernal)
    res2 = cv2.bitwise_and(img, img, mask=green)

    # Tracking the Red Color
    img1, contours, hierarchy = cv2.findContours(red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    config.set_traffic('green')
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 2500):
            x, y, w, h = cv2.boundingRect(contour)
            img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(img, "RED color", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))
            config.set_traffic('red')
            # print('stop')

    # Tracking the green Color
    img1, contours, hierarchy = cv2.findContours(green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 2500):
            x, y, w, h = cv2.boundingRect(contour)
            img = cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img, "Green  color", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0))
            print('green')
            # time.sleep(1)
            # print('fwd')

    # cv2.imshow("Redcolour", red)
    cv2.imshow("Color Tracking", img)
    cv2.moveWindow('Color Tracking', 80, 850)
    # cv2.imshow("red", res)
    #cv2.imshow('green',green)

