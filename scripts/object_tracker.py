#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray, Int16
import cv2
import numpy as np
import time

def publishObjectData(horizontalPos, size):
	#print("Size: {}", size)
	#print("Pos: {}", horizontalPos)
	global stampId
	stampId += 1
	#print stampId, " Published from tracker at: ", int(round(time.time() * 1000))
	data = Int16MultiArray()
	data.data = [horizontalPos, size]
	objectPub.publish(data)

def publishNewObject(size):
	print "Publishing new object"
	data = Int16()
	data.data = size
	newObjectPub.publish(data)

def publishPowerData():
	data = Int16MultiArray()
	data.data = [0, 0]
	powerPub.publish(data)

def startNewObject(x):
	print("Starting new object")
	global newObject
	newObject = True

def nothing(x):
	pass

try:
	rospy.init_node('object_tracker', anonymous=True)
	objectPub = rospy.Publisher('/zumo/object_data', Int16MultiArray, queue_size=1)
	powerPub = rospy.Publisher('/zumo/power', Int16MultiArray, queue_size=1)
	newObjectPub = rospy.Publisher('/zumo/new_object', Int16, queue_size=1)

	#Capture default video stream
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
	cv2.createTrackbar(trackingTitle, "Tracking", 0, 1, startNewObject)

	stamp = int(round(time.time() * 1000))
	stampId = 0

	newObject = True

	while True:
		if not int(round(time.time() * 1000)) >= stamp:
			continue
		#print "Time:  ", int(round(time.time() * 1000))
		#print "Stamp: ", stamp		

		_, frame = cap.read()

		#Blur image: This was too computationally expensive on the pi to run at a useful level. 
		#blurred = cv2.GaussianBlur(frame, (25, 25), cv2.BORDER_DEFAULT)

		#Convert BGR image to HSV
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		lowerHue = cv2.getTrackbarPos("Lower Hue", "Tracking")
		upperHue = cv2.getTrackbarPos("Upper Hue", "Tracking")
		lowerSaturation = cv2.getTrackbarPos("Lower Saturation", "Tracking")
		upperSaturation = cv2.getTrackbarPos("Upper Saturation", "Tracking")
		lowerValue = cv2.getTrackbarPos("Lower Value", "Tracking")	
		upperValue = cv2.getTrackbarPos("Upper Value", "Tracking")	
		isTracking = cv2.getTrackbarPos(trackingTitle, "Tracking")	
		
		lowerBounds = np.array([lowerHue, lowerSaturation, lowerValue])
		upperBounds = np.array([upperHue, upperSaturation, upperValue])

		#Generate B&W mask based on upper and lower bounds applied to frame
		mask = cv2.inRange(hsv, lowerBounds, upperBounds)

		#AND the mask with frame to produce isolated visual for GUI
		result = cv2.bitwise_and(frame, frame, mask=mask)

		#Get pixel bounds of non-zero area
		nz = cv2.findNonZero(mask)
		if (not nz is None):
			left = nz[:, 0, 0].min()
			right = nz[:, 0, 0].max()
			upper = nz[:, 0, 1].min()
			lower = nz[:, 0, 1].max()
						
			#Draw rectangle around non-zero area
			cv2.rectangle(frame, (left, upper), (right, lower), (0, 255, 0)) 

			if (isTracking == 1):
				horizontalPos = (left+right)/2.0
				#Use the longest visible side as the measure of distance, using the larger value will help avoid collision
				size = max(right-left, lower-upper)
				#print "Size: ", size, " Position: ", horizontalPos
				if (newObject):
					publishNewObject(size)
					newObject = False
				publishObjectData(horizontalPos, size)
		
		stack = np.hstack((frame, result))
		#Display outputs
		cv2.imshow("Tracking", stack)
		key = cv2.waitKey(1)
		if key == 27:
			break

		#Add 100 ms to timestamp, running loop 10 times per second
		stamp += 100

		#If tracking is off, send 0 power values to the zumo		
		if isTracking == 0:
			publishPowerData()


except KeyboardInterrupt:
	cap.release()
	cv2.destroyAllWindows()
