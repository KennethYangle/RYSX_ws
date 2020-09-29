#!/usr/bin/python
import sys
import cv2
import numpy as np

def print_hsv(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(hue_image[y,x])

if len(sys.argv) != 2:
    print("Usage: {0} PICTURE_PATH".format(sys.argv[0]))
    sys.exit(-1)
cv2.namedWindow('image')
cv2.setMouseCallback('image', print_hsv)
im = cv2.imread(sys.argv[1])
hue_image = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
cv2.imshow("image", im)
cv2.waitKey()