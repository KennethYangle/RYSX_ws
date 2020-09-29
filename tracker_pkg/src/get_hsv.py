#!/usr/bin/python
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
#import numpy as np


low = [255,255,255]
high = [0,0,0]

def print_hsv(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        lowhighthsv(x,y)
        print('temp', hue_image[y,x])
        print('low',low)
        print('high', high)

def lowhighthsv(x,y):
    global low, high
    temp = hue_image[y,x]
    if temp[0]<low[0]:
        low[0] = temp[0]
    if temp[0]>high[0]:
        high[0] = temp[0]
    if temp[1]<low[1]:
        low[1] = temp[1]
    if temp[1]>high[1]:
        high[1] = temp[1]
    if temp[2]<low[2]:
        low[2] = temp[2]
    if temp[2]>high[2]:
        high[2] = temp[2]

cv2.namedWindow('image')
cv2.setMouseCallback('image', print_hsv)
cap = cv2.VideoCapture("/home/zhou/Desktop/1.mp4")
while True:
    ret, frame = cap.read()
    hue_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.imshow("image", frame)
#读取内容
    if cv2.waitKey(300) == ord("q"):
        break
        
#随时准备按q退出
cap.release()
cv2.destroyAllWindows()



