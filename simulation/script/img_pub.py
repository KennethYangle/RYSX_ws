#!/usr/bin/env python
#coding=utf-8

import numpy as np
import rospy
#导入自定义的数据类型
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import HomePosition

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from gazebo_msgs.srv import *
import threading
from utils import Utils
from Queue import Queue

img_x = 0
img_y = 0
img_size = 0

# imag_pub = rospy.Publisher("tracker/img_pos", Vector3Stamped, queue_size=10)  # 发送图像位置

def image_callback(data):
    # define picture to_down' coefficient of ratio
    scaling_factor = 0.5
    global count,bridge
    count = count + 1

    img_pos = Vector3Stamped()

    if count == 1:
        count = 0
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")

        gay_img = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2GRAY)
        img = cv2.medianBlur(gay_img, 7)  # 进行中值模糊，去噪点
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 120, param1=100, param2=30, minRadius=0, maxRadius=100)


        # print(circles)
        img_x = 0
        img_y = 0
        img_size = 0

        if not  circles is None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:  # 遍历矩阵每一行的数据
                cv2.circle(cv_img, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(cv_img, (i[0], i[1]), 2, (0, 0, 255), 3)

            img_x = i[0]
            img_y = i[1]
            img_size = 3.14 * (i[2]) ** 2

            print(img_size)

            img_pos.vector.x = img_x
            img_pos.vector.y = img_y
            img_pos.vector.z = img_size

            imag_pub.publish(img_pos)
            cv2.imshow("frame", cv_img)
            cv2.waitKey(3)
        else:
            img_pos.vector.x = img_x
            img_pos.vector.y = img_y
            img_pos.vector.z = img_size
            imag_pub.publish(img_pos)
            print(img_size)
    else:
        pass


if __name__ == '__main__':
    global imag_pub
    imag_pub = rospy.Publisher("tracker/img_pos", Vector3Stamped, queue_size=10)  # 发送图像位置
    rospy.init_node('iris_fpv_cam', anonymous=True)

    # img_pos = Vector3Stamped()

    global count, bridge
    count = 0
    bridge = CvBridge()
    rospy.Subscriber('/iris_fpv_cam/usb_cam/image_raw', Image, image_callback)

    rospy.spin()





