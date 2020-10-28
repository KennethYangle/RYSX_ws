#!/usr/bin/env python
#coding=utf-8

import numpy as np
import os, json
import rospy
#导入自定义的数据类型
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from gazebo_msgs.srv import *

cnt = 0
def image_callback(data):
    # define picture to_down' coefficient of ratio
    scaling_factor = 0.5
    global bridge, cnt, params
    cnt += 1

    img_pos = Float32MultiArray()
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    hue_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
    low_range = np.array([0, 200, 50])
    high_range = np.array([5, 256, 256])
    th = cv2.inRange(hue_image, low_range, high_range)
    dilated = cv2.dilate(th, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)
    cv2.imshow("hsv", dilated)
    cv2.waitKey(1)
    M = cv2.moments(dilated, binaryImage = True)
    if M["m00"] > 100:
        img_pos.data = [int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), np.sqrt(2*M["m00"]), np.sqrt(2*M["m00"]), 0.8]
    else:
        img_pos.data = [0, 0, 0, 0, 0]

    if cnt > params["cam_lose_cnt"]:
        img_pos.data = [0, 0, 0, 0, 0]
    imag_pub.publish(img_pos)


if __name__ == '__main__':
    global params
    setting_file = open(os.path.join(os.path.expanduser('~'),"RYSX_ws/src","settings.json"))
    setting = json.load(setting_file)
    params = setting["Simulation"]

    global imag_pub
    imag_pub = rospy.Publisher("tracker/pos_image", Float32MultiArray, queue_size=10)  # 发送图像位置
    rospy.init_node('iris_fpv_cam', anonymous=True)

    global bridge
    bridge = CvBridge()
    rospy.Subscriber('/iris/usb_cam/image_raw', Image, image_callback)

    rospy.spin()
