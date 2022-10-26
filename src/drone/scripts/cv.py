#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from drone.oceanikaAPI import UUV
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

cv_pub = rospy.Publisher("/cv/image", Image, queue_size=1)


def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

def getContours(img, imgContour):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    for cnt in contours:
        area = cv2.contourArea(cnt)  # This is used to find the area of the contour.
        print("Area: ", area)
        if area > 1000:  # The areas below 500 pixels will not be considered
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 7)  # -1 denotes that we need to draw all the contours
            perimeter = cv2.arcLength(cnt, True)  # The true indicates that the contour is closed
            print("Perimeter: ", perimeter)
            approx = cv2.approxPolyDP(cnt, 0.02 * perimeter,
                                      True)  # This method is used to find the approximate number of contours
            print("Corner Points: ", len(approx))
            objCorner = len(approx)
            x, y, w, h = cv2.boundingRect(
                approx)  # In this we get the values of our bounding box that we will draw around the object

            if objCorner == 3:
                objectType = 'Triangle'
            elif objCorner == 4:
                aspectRatio = float(w) / float(h)
                if aspectRatio > 0.95 and aspectRatio < 1.05:
                    objectType = 'Square'
                else:
                    objectType = "Rectangle"
            elif objCorner > 4:
                objectType = 'Circle'
            else:
                objectType = "None"

            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw a rectange around the shapes
            cv2.putText(imgContour, objectType, (x + (w // 2) - 10, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 0.5,
                        (0, 0, 0), 2)

def cb(img_msg):
    rospy.loginfo(img_msg.header)

    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))


    threshold1 = cv2.getTrackbarPos("Threshold1", "Parameters")
    threshold2 = cv2.getTrackbarPos("Threshold2", "Parameters")

    cv_image_contour = cv_image.copy()
    cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    cv_image_blur = cv2.GaussianBlur(cv_image_gray, (7, 7), 1)
    cv_image_canny = cv2.Canny(cv_image_blur, threshold1, threshold2)
    kernel = np.ones((5, 5))
    cv_image_dil = cv2.dilate(cv_image_canny, kernel, iterations=1)

    getContours(cv_image_dil, cv_image_contour)

    # cv_image_stack = stackImages(.8, ([cv_image, cv_image_gray, cv_image_canny], [cv_image_dil, cv_image_contour, cv_image_contour]))
    # ret, thresh = cv2.threshold(cv_image_gray, 127, 255, 0)
    # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    image_message = bridge.cv2_to_imgmsg(cv_image_contour, encoding="bgr8")
    cv_pub.publish(image_message)
    # show_image(cv_image_stack)

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

if __name__ == "__main__":
    rospy.init_node('opencv_example')

    cv2.namedWindow("Image Window", 1)
    cv2.namedWindow("Parameters")
    cv2.resizeWindow("Parameters", 640, 240)
    cv2.createTrackbar("Threshold1", "Parameters", 46, 255, empty)
    cv2.createTrackbar("Threshold2", "Parameters", 16, 255, empty)

    img = rospy.Subscriber("/raspicam_node/image_raw", Image, cb)
    while not rospy.is_shutdown():
        rospy.spin()