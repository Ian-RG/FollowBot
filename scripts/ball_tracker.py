#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray
import cv2
import numpy as np
import time

#55 at 50cm
#Ball is 90 pixels wide at 30cm
#210 at 10cm
#centre is 300

def publishData(horizontalPos, size):
	#print("Size: {}", size)
	#print("Pos: {}", horizontalPos)
	data = Int16MultiArray()
	data.data = [horizontalPos, size]
	pub.publish(data)

try:
	rospy.init_node('ball_tracker', anonymous=True)
	pub = rospy.Publisher('/zumo/ball_pos', Int16MultiArray, queue_size=10)

	#Capture default video stream
	cap = cv2.VideoCapture(0)

	stamp = int(round(time.time() * 1000))

	while True:
		if not int(round(time.time() * 1000)) >= stamp:
			continue
		print "Time:  ", int(round(time.time() * 1000))
		print "Stamp: ", stamp
		_, frame = cap.read()

		#Blur image
		frame = cv2.GaussianBlur(frame, (15, 15), cv2.BORDER_DEFAULT)

		#Convert BGR image to HSV
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		#Set lower and upper HSV values associated with target object
		lowerBounds = np.array([38, 40, 54])
		upperBounds = np.array([57, 100, 138])

		#Generate B&W mask based on upper and lower bounds applied to frame
		mask = cv2.inRange(hsv, lowerBounds, upperBounds)

		#AND the mask with frame to produce isolated visual (debug)
		#result = cv2.bitwise_and(frame, frame, mask=mask)

		#Get pixel bounds of non-zero area
		nz = cv2.findNonZero(mask)
		if (not nz is None):
			left = nz[:, 0, 0].min()
			right = nz[:, 0, 0].max()
			upper = nz[:, 0, 1].min()
			lower = nz[:, 0, 1].max()

			horizontalPos = (left+right)/2.0
			#Use the longest visible side as the measure of distance
			#We're chasing a ball, so ideally the shape will be a square.
			#If it's partially off-camera, using the larger value will help avoid collision
			size = max(left-right, lower-upper)

			publishData(horizontalPos, size)
			#Draw rectangle around non-zero area
			#cv2.rectangle(frame, (left, upper), (right, lower), (0, 255, 0)) 
		
		#Display outputs (debug)
		#cv2.imshow("frame", frame)
		#cv2.imshow("mask", mask)
		#cv2.imshow("result", result)
		#key = cv2.waitKey(1)
		#if key == 27:
		#	break

		#Add 200 ms to timestamp, running loop 5 times per second
		stamp += 200

except KeyboardInterrupt:
	cap.release()
	cv2.destroyAllWindows()

	


	
