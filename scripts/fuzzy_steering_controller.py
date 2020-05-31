#!/usr/bin/env python
from __future__ import division
import time
import rospy
from std_msgs.msg import Int16, Int16MultiArray
from fuzzy_functions import FuzzyTrapezoid, FuzzyTriangle


rospy.init_node('fuzzy_steering_controller', anonymous=True)
pub = rospy.Publisher('/zumo/power', Int16MultiArray, queue_size = 1)

#Size of target
targetUndersizeBigFn = FuzzyTrapezoid(0, 0, 50, 70)
targetUndersizeSmallFn = FuzzyTriangle(50, 70, 90)
targetIdealSizeFn = FuzzyTriangle(70, 90, 110)
targetOversizeSmallFn = FuzzyTriangle(90, 110, 130)
targetOversizeBigFn = FuzzyTrapezoid(110, 130, 500, 500)

#Ball position relative to centre of frame
leftBigFn = FuzzyTrapezoid(-100, -100, 100, 200)
leftSmallFn = FuzzyTriangle(100, 200, 300)
centreFn = FuzzyTriangle(250, 300, 350)
rightSmallFn = FuzzyTriangle(300, 400, 500)
rightBigFn = FuzzyTrapezoid(400, 500, 1000, 1000)

HARD_TURN_LEFT = -1
MED_HARD_TURN_LEFT = -0.75
MED_SOFT_TURN_LEFT = -0.5
SOFT_TURN_LEFT = -0.25
NO_TURN = 0
SOFT_TURN_RIGHT = 0.25
MED_SOFT_TURN_RIGHT = 0.5
MED_HARD_TURN_RIGHT = 0.75
HARD_TURN_RIGHT = 1

stampId = 0

def publishData(leftPower, rightPower):
	global stampId
	stampId += 1
	print stampId, " Published from steering at: ", int(round(time.time() * 1000))
	data = Int16MultiArray()
	data.data = [leftPower, rightPower]
	pub.publish(data)


def adjustPowerForTurning(power, ballPosition, ballDimension):	
	closeBig = targetOversizeBigFn.getMembership(ballDimension)
	closeSmall = targetOversizeSmallFn.getMembership(ballDimension)
	atTarget = targetIdealSizeFn.getMembership(ballDimension)
	farSmall = targetUndersizeSmallFn.getMembership(ballDimension)
	farBig = targetUndersizeBigFn.getMembership(ballDimension)

	rightBig = rightBigFn.getMembership(ballPosition)
	rightSmall = rightSmallFn.getMembership(ballPosition)
	centre = centreFn.getMembership(ballPosition)
	leftSmall = leftSmallFn.getMembership(ballPosition)
	leftBig = leftBigFn.getMembership(ballPosition)

	memberships = []
	if closeBig > 0:		
		if rightBig > 0: memberships.append([(closeBig+rightBig)/2, MED_HARD_TURN_RIGHT])
		if rightSmall > 0: memberships.append([(closeBig+rightSmall)/2, MED_HARD_TURN_RIGHT])
		if centre > 0: memberships.append([(closeBig+centre)/2, NO_TURN])
		if leftSmall > 0: memberships.append([(closeBig+leftSmall)/2, MED_HARD_TURN_LEFT])
		if leftBig > 0: memberships.append([(closeBig+leftBig)/2, MED_HARD_TURN_LEFT])
		
	if (closeSmall > 0):
		if rightBig > 0: memberships.append([(closeSmall+rightBig)/2, MED_HARD_TURN_RIGHT])
		if rightSmall > 0: memberships.append([(closeSmall+rightSmall)/2, MED_SOFT_TURN_RIGHT])
		if centre > 0: memberships.append([(closeSmall+centre)/2, NO_TURN])
		if leftSmall > 0: memberships.append([(closeSmall+leftSmall)/2, MED_HARD_TURN_LEFT])
		if leftBig > 0: memberships.append([(closeSmall+leftBig)/2, MED_SOFT_TURN_LEFT])

	if (atTarget > 0): 
		if rightBig > 0: memberships.append([(atTarget+rightBig)/2, MED_HARD_TURN_RIGHT])
		if rightSmall > 0: memberships.append([(atTarget+rightSmall)/2, MED_SOFT_TURN_RIGHT])
		if centre > 0: memberships.append([(atTarget+centre)/2, NO_TURN])
		if leftSmall > 0: memberships.append([(atTarget+leftSmall)/2, MED_SOFT_TURN_LEFT])
		if leftBig > 0: memberships.append([(atTarget+leftBig)/2, MED_HARD_TURN_LEFT])

	if (farSmall > 0): 
		if rightBig > 0: memberships.append([(farSmall+rightBig)/2, MED_SOFT_TURN_RIGHT])
		if rightSmall > 0: memberships.append([(farSmall+rightSmall)/2, SOFT_TURN_RIGHT])
		if centre > 0: memberships.append([(farSmall+centre)/2, NO_TURN])
		if leftSmall > 0: memberships.append([(farSmall+leftSmall)/2, SOFT_TURN_LEFT])
		if leftBig > 0: memberships.append([(farSmall+leftBig)/2, MED_SOFT_TURN_LEFT])

	if (farBig > 0): 
		if rightBig > 0: memberships.append([(farBig+rightBig)/2, SOFT_TURN_RIGHT])
		if rightSmall > 0: memberships.append([(farBig+rightSmall)/2, SOFT_TURN_RIGHT])
		if centre > 0: memberships.append([(farBig+centre)/2, NO_TURN])
		if leftSmall > 0: memberships.append([(farBig+leftSmall)/2, SOFT_TURN_LEFT])
		if leftBig > 0: memberships.append([(farBig+leftBig)/2, SOFT_TURN_LEFT])

	weightedAreaSum = 0
	areaSum = 0

	turnRatio = 0

	for m in memberships:
		m[0] = 0.5 * m[0] * (1 - (m[0]/2))
		weightedAreaSum += m[0] * m[1]
		areaSum += m[0]

	if (areaSum != 0):
		turnRatio = weightedAreaSum / areaSum

	#When reversing, the track opposite the turn needs to be slowed (ie slow the right track for a left turn when reversing)
	leftPower = rightPower = power
	if (turnRatio < 0):
		if (power > 0): leftPower *= 1+turnRatio
		else: rightPower *= 1+turnRatio
	elif (turnRatio > 0):
		if (power > 0): rightPower *= 1-turnRatio
		else: leftPower *= 1-turnRatio
	
	#print turnRatio
		
	return leftPower, rightPower


def callback(data):
	ballPosition = data.data[0]
	ballDimension = data.data[1]
	power = data.data[2]

	leftPower, rightPower = adjustPowerForTurning(power, ballPosition, ballDimension)

	#print "Ball position: ", ballPosition, " Ball dimension: ", ballDimension, "Left Power: ", leftPower, "Right Power: ", rightPower
	publishData(leftPower, rightPower)

def listener():
	rospy.Subscriber('/zumo/raw_power', Int16MultiArray, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
