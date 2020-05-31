#!/usr/bin/env python
from __future__ import division
import time
import rospy
from std_msgs.msg import Int16MultiArray
from fuzzy_functions import FuzzyTrapezoid, FuzzyTriangle

rospy.init_node('fuzzy_power_controller', anonymous=True)
pub = rospy.Publisher('/zumo/raw_power', Int16MultiArray, queue_size = 1)

#Size of target
targetUndersizeBigFn = FuzzyTrapezoid(0, 0, 50, 70)
targetUndersizeSmallFn = FuzzyTriangle(50, 70, 90)
targetIdealSizeFn = FuzzyTriangle(70, 90, 110)
targetOversizeSmallFn = FuzzyTriangle(90, 110, 130)
targetOversizeBigFn = FuzzyTrapezoid(110, 130, 500, 500)

#Rate of change in relation to target between readings
deceleratingBigFn = FuzzyTrapezoid(-200, -200, -2, -1)
deceleratingSmallFn = FuzzyTriangle(-2, -1, 0)
speedConstantFn = FuzzyTriangle(-1, 0, 2)
acceleratingSmallFn = FuzzyTriangle(0, 2, 4)
acceleratingBigFn = FuzzyTrapezoid(2, 4, 200, 200)

BIG_SPEED_CHANGE = 16
SMALL_SPEED_CHANGE = 3
NO_SPEED_CHANGE = 0
POWER_LIMIT = 120

power = 0
previousBallDimension = 0

stampId = 0

def publishData(power, ballPosition, ballDimension):
	global stampId
	stampId += 1
	print stampId, " Published from power at: ", int(round(time.time() * 1000))
	data = Int16MultiArray()
	data.data = [ballPosition, ballDimension, power]
	pub.publish(data)

def getPower(power, ballDimension, deltaV):
	closeBig = targetOversizeBigFn.getMembership(ballDimension)
	closeSmall = targetOversizeSmallFn.getMembership(ballDimension)
	atTarget = targetIdealSizeFn.getMembership(ballDimension)
	farSmall = targetUndersizeSmallFn.getMembership(ballDimension)
	farBig = targetUndersizeBigFn.getMembership(ballDimension)

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


def callback(data):
	global power
	global previousBallDimension
	ballPosition = data.data[0]
	ballDimension = data.data[1]
	#Change in velocity relative to target
	deltaV = ballDimension - previousBallDimension
	print deltaV

	power = getPower(power, ballDimension, deltaV)

	#print "Ball position: ", ballPosition, " Ball dimension: ", ballDimension, "Power: ", power
	publishData(power, ballPosition, ballDimension)
	previousBallDimension = ballDimension

def listener():
	rospy.Subscriber('/zumo/ball_pos', Int16MultiArray, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
