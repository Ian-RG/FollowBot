#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from std_msgs.msg import Int16MultiArray

def nothing(x):
	pass

cap = cv2.VideoCapture(0)

cv2.namedWindow("Tracking")
#Create trackbars for setting values. Initialize lower boundaries to 0, uppers to 255
cv2.createTrackbar("Lower Hue", "Tracking", 0, 255, nothing)
cv2.createTrackbar("Upper Hue", "Tracking", 255, 255, nothing)
cv2.createTrackbar("Lower Saturation", "Tracking", 0, 255, nothing)
cv2.createTrackbar("Upper Saturation", "Tracking", 255, 255, nothing)
cv2.createTrackbar("Lower Value", "Tracking", 0, 255, nothing)
cv2.createTrackbar("Upper Value", "Tracking", 255, 255, nothing)

trackingTitle = "Tracking:\n0 - Off\n1 - On"
cv2.createTrackbar(trackingTitle, "Tracking", 0, 1, nothing)

while True:
	#frame = cv2.imread('image.jpg')
	_, frame = cap.read()

	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	#lower and upper limits for hsv image
	lowerHue = cv2.getTrackbarPos("Lower Hue", "Tracking")
	upperHue = cv2.getTrackbarPos("Upper Hue", "Tracking")
	lowerSaturation = cv2.getTrackbarPos("Lower Saturation", "Tracking")
	upperSaturation = cv2.getTrackbarPos("Upper Saturation", "Tracking")
	lowerValue = cv2.getTrackbarPos("Lower Value", "Tracking")	
	upperValue = cv2.getTrackbarPos("Upper Value", "Tracking")
	
	isTracking = cv2.getTrackbarPos(trackingTitle, "Tracking")
	if isTracking == 1:
		#publish
		pass

	lowerBounds = np.array([lowerHue, lowerSaturation, lowerValue])
	upperBounds = np.array([upperHue, upperSaturation, upperValue])

	mask = cv2.inRange(hsv, lowerBounds, upperBounds)

	result = cv2.bitwise_and(frame, frame, mask=mask)

	nz = cv2.findNonZero(mask)
	if (not nz is None):
		upper = nz[:, 0, 0].min()
		lower = nz[:, 0, 0].max()
		left = nz[:, 0, 1].min()
		right = nz[:, 0, 1].max()

		cv2.rectangle(frame, (upper, left), (lower, right), (0, 255, 0)) 

	stack = np.hstack((frame, result))
	
	cv2.imshow("Tracking", stack)

	key = cv2.waitKey(1)
	if key == 27:
		break

cap.release()
cv2.destroyAllWindows()
