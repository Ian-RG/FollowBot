#!/usr/bin/env python
from __future__ import division
import time
import rospy
from std_msgs.msg import Int16MultiArray, Int16
from fuzzy_functions import FuzzyTrapezoid, FuzzyTriangle

rospy.init_node('fuzzy_power_controller', anonymous=True)
pub = rospy.Publisher('/zumo/raw_power', Int16MultiArray, queue_size = 1)


#Size of target
targetUndersizeBigFn = FuzzyTrapezoid(0, 0, 0, 0)
targetUndersizeSmallFn = FuzzyTriangle(0, 0, 0)
targetIdealSizeFn = FuzzyTriangle(0, 0, 0)
targetOversizeSmallFn = FuzzyTriangle(0, 0, 0)
targetOversizeBigFn = FuzzyTrapezoid(0, 0, 0, 0)

#Rate of change in relation to target between readings
deceleratingBigFn = FuzzyTrapezoid(-200, -200, -2, -1)
deceleratingSmallFn = FuzzyTriangle(-2, -1, 0)
speedConstantFn = FuzzyTriangle(-1, 0, 1)
acceleratingSmallFn = FuzzyTriangle(0, 1, 2)
acceleratingBigFn = FuzzyTrapezoid(1, 2, 200, 200)

BIG_SPEED_CHANGE = 6
SMALL_SPEED_CHANGE = 3
NO_SPEED_CHANGE = 0
POWER_LIMIT = 120

power = 0
previousObjectDimension = 0

stampId = 0

def publishData(power, objectPosition, objectDimension):
	global stampId
	stampId += 1
	#print stampId, " Published from power at: ", int(round(time.time() * 1000))
	data = Int16MultiArray()
	data.data = [objectPosition, objectDimension, power]
	pub.publish(data)

def getPower(power, objectDimension, deltaV):
	closeBig = targetOversizeBigFn.getMembership(objectDimension)
	closeSmall = targetOversizeSmallFn.getMembership(objectDimension)
	atTarget = targetIdealSizeFn.getMembership(objectDimension)
	farSmall = targetUndersizeSmallFn.getMembership(objectDimension)
	farBig = targetUndersizeBigFn.getMembership(objectDimension)

	deceleratingBig = deceleratingBigFn.getMembership(deltaV)
	deceleratingSmall = deceleratingSmallFn.getMembership(deltaV)
	speedConstant = speedConstantFn.getMembership(deltaV)
	acceleratingSmall = acceleratingSmallFn.getMembership(deltaV)
	acceleratingBig = acceleratingBigFn.getMembership(deltaV)

	#print "closeBig:   ", closeBig
	#print "closeSmall: ", closeSmall
	#print "atTarget:   ", atTarget
	#print "farSmall:   ", farSmall
	#print "farBig:     ", farBig
	#print ""
	#print "deceleratingBig:   ", deceleratingBig
	#print "deceleratingSmall: ", deceleratingSmall
	#print "speedConstant:  ", speedConstant
	#print "acceleratingSmall:   ", acceleratingSmall
	#print "acceleratingBig:     ", acceleratingBig

	memberships = []
	if closeBig > 0:		
		if deceleratingBig > 0: memberships.append([(closeBig+deceleratingBig)/2, -SMALL_SPEED_CHANGE])
		if deceleratingSmall > 0: memberships.append([(closeBig+deceleratingSmall)/2, -BIG_SPEED_CHANGE])
		if speedConstant > 0: memberships.append([(closeBig+speedConstant)/2, -BIG_SPEED_CHANGE])
		if acceleratingSmall > 0: memberships.append([(closeBig+acceleratingSmall)/2, -BIG_SPEED_CHANGE])
		if acceleratingBig > 0: memberships.append([(closeBig+acceleratingBig)/2, -BIG_SPEED_CHANGE])
		
	if (closeSmall > 0):
		if deceleratingBig > 0: memberships.append([(closeSmall+deceleratingBig)/2, SMALL_SPEED_CHANGE])
		if deceleratingSmall > 0: memberships.append([(closeSmall+deceleratingSmall)/2, NO_SPEED_CHANGE])
		if speedConstant > 0: memberships.append([(closeSmall+speedConstant)/2, -SMALL_SPEED_CHANGE])
		if acceleratingSmall > 0: memberships.append([(closeSmall+acceleratingSmall)/2, -BIG_SPEED_CHANGE])
		if acceleratingBig > 0: memberships.append([(closeSmall+acceleratingBig)/2, -BIG_SPEED_CHANGE])

	if (atTarget > 0): 
		if deceleratingBig > 0: memberships.append([(atTarget+deceleratingBig)/2, BIG_SPEED_CHANGE])
		if deceleratingSmall > 0: memberships.append([(atTarget+deceleratingSmall)/2, SMALL_SPEED_CHANGE])
		if speedConstant > 0: memberships.append([(atTarget+speedConstant)/2, NO_SPEED_CHANGE])
		if acceleratingSmall > 0: memberships.append([(atTarget+acceleratingSmall)/2, -SMALL_SPEED_CHANGE])
		if acceleratingBig > 0: memberships.append([(atTarget+acceleratingBig)/2, -BIG_SPEED_CHANGE])

	if (farSmall > 0): 
		if deceleratingBig > 0: memberships.append([(farSmall+deceleratingBig)/2, BIG_SPEED_CHANGE])
		if deceleratingSmall > 0: memberships.append([(farSmall+deceleratingSmall)/2, BIG_SPEED_CHANGE])
		if speedConstant > 0: memberships.append([(farSmall+speedConstant)/2, SMALL_SPEED_CHANGE])
		if acceleratingSmall > 0: memberships.append([(farSmall+acceleratingSmall)/2, NO_SPEED_CHANGE])
		if acceleratingBig > 0: memberships.append([(farSmall+acceleratingBig)/2, -SMALL_SPEED_CHANGE])

	if (farBig > 0): 
		if deceleratingBig > 0: memberships.append([(farBig+deceleratingBig)/2, BIG_SPEED_CHANGE])
		if deceleratingSmall > 0: memberships.append([(farBig+deceleratingSmall)/2, BIG_SPEED_CHANGE])
		if speedConstant > 0: memberships.append([(farBig+speedConstant)/2, BIG_SPEED_CHANGE])
		if acceleratingSmall > 0: memberships.append([(farBig+acceleratingSmall)/2, BIG_SPEED_CHANGE])
		if acceleratingBig > 0: memberships.append([(farBig+acceleratingBig)/2, SMALL_SPEED_CHANGE])

	weightedAreaSum = 0
	areaSum = 0

	for m in memberships:
		m[0] = 6 * m[0] * (1 - (m[0]/2))
		weightedAreaSum += m[0] * m[1]
		areaSum += m[0]

	if (areaSum != 0):
		powerChange = weightedAreaSum / areaSum
		#print "Powerchange: ", powerChange
		power += powerChange
		

	if (power > POWER_LIMIT):
		power = POWER_LIMIT
	if (power < -POWER_LIMIT):
		power = -POWER_LIMIT	
	return power


def objectDataCallback(data):
	global power
	global previousObjectDimension
	objectPosition = data.data[0]
	objectDimension = data.data[1]
	#Change in velocity relative to target
	deltaV = objectDimension - previousObjectDimension
	#print deltaV

	power = getPower(power, objectDimension, deltaV)

	#print "Object position: ", objectPosition, " Object dimension: ", objectDimension, "Power: ", power
	publishData(power, objectPosition, objectDimension)
	previousObjectDimension = objectDimension

def newObjectCallback(data):
	width = data.data
	global targetUndersizeBigFn, targetUndersizeSmallFn, targetIdealSizeFn, targetOversizeSmallFn, targetOversizeSmallFn
	targetUndersizeBigFn = FuzzyTrapezoid(0, 0, width*0.55, width*0.8)
	targetUndersizeSmallFn = FuzzyTriangle(width*0.55, width*0.8, width)
	targetIdealSizeFn = FuzzyTriangle(width*0.8, width, width*1.2)
	targetOversizeSmallFn = FuzzyTriangle(width, width*1.2, width*1.45)
	targetOversizeBigFn = FuzzyTrapezoid(width*1.2, width*1.45, 1000, 1000)

def listener():
	rospy.Subscriber('/zumo/object_data', Int16MultiArray, objectDataCallback)
	rospy.Subscriber('/zumo/new_object', Int16, newObjectCallback)
	rospy.spin()

if __name__ == '__main__':
	listener()
