#!/usr/bin/env python
from __future__ import division
import time
import rospy
from std_msgs.msg import Int16, Int16MultiArray
from fuzzy_functions import FuzzyTrapezoid, FuzzyTriangle


rospy.init_node('fuzzy_steering_controller', anonymous=True)
pub = rospy.Publisher('/zumo/power', Int16MultiArray, queue_size = 1)

#Lateral movement of target
movingLeftBigFn = FuzzyTrapezoid(-200, -200, -20, -10)
movingLeftSmallFn = FuzzyTriangle(-20, -10, 0)
notMovingFn = FuzzyTriangle(-10, 0, 10)
movingRightSmallFn = FuzzyTriangle(0, 10, 20)
movingRightBigFn = FuzzyTrapezoid(10, 20, 200, 200)

#Object position relative to centre of frame
leftBigFn = FuzzyTrapezoid(-100, -100, 100, 200)
leftSmallFn = FuzzyTriangle(100, 200, 300)
centreFn = FuzzyTriangle(250, 300, 350)
rightSmallFn = FuzzyTriangle(300, 400, 500)
rightBigFn = FuzzyTrapezoid(400, 500, 1000, 1000)

HARD_LEFT_TURN = -0.1
SOFT_LEFT_TURN = -0.05
NO_TURN = 0
SOFT_RIGHT_TURN = 0.05
HARD_RIGHT_TURN = 0.1

stampId = 0

previousObjectPosition = 0
turnRatio = 0

def publishData(leftPower, rightPower):
	global stampId
	stampId += 1
	#print stampId, " Published from steering at: ", int(round(time.time() * 1000))
	data = Int16MultiArray()
	data.data = [leftPower, rightPower]
	pub.publish(data)


def adjustPowerForTurning(power, objectPosition, lateralDeltaV):	
	movingRightBig = movingRightBigFn.getMembership(lateralDeltaV)
	movingRightSmall = movingRightSmallFn.getMembership(lateralDeltaV)
	notMoving = notMovingFn.getMembership(lateralDeltaV)
	movingLeftSmall = movingLeftSmallFn.getMembership(lateralDeltaV)
	movingLeftBig = movingLeftBigFn.getMembership(lateralDeltaV)

	rightBig = rightBigFn.getMembership(objectPosition)
	rightSmall = rightSmallFn.getMembership(objectPosition)
	centre = centreFn.getMembership(objectPosition)
	leftSmall = leftSmallFn.getMembership(objectPosition)
	leftBig = leftBigFn.getMembership(objectPosition)

	memberships = []
	if movingRightBig > 0:		
		if rightBig > 0: memberships.append([(movingRightBig+rightBig)/2, HARD_RIGHT_TURN])
		if rightSmall > 0: memberships.append([(movingRightBig+rightSmall)/2, HARD_RIGHT_TURN])
		if centre > 0: memberships.append([(movingRightBig+centre)/2, SOFT_RIGHT_TURN])
		if leftSmall > 0: memberships.append([(movingRightBig+leftSmall)/2, SOFT_RIGHT_TURN])
		if leftBig > 0: memberships.append([(movingRightBig+leftBig)/2, SOFT_RIGHT_TURN])
		
	if (movingRightSmall > 0):
		if rightBig > 0: memberships.append([(movingRightSmall+rightBig)/2, HARD_RIGHT_TURN])
		if rightSmall > 0: memberships.append([(movingRightSmall+rightSmall)/2, SOFT_RIGHT_TURN])
		if centre > 0: memberships.append([(movingRightSmall+centre)/2, SOFT_RIGHT_TURN])
		if leftSmall > 0: memberships.append([(movingRightSmall+leftSmall)/2, NO_TURN])
		if leftBig > 0: memberships.append([(movingRightSmall+leftBig)/2, SOFT_RIGHT_TURN])

	if (notMoving > 0): 
		if rightBig > 0: memberships.append([(notMoving+rightBig)/2, SOFT_RIGHT_TURN])
		if rightSmall > 0: memberships.append([(notMoving+rightSmall)/2, SOFT_RIGHT_TURN])
		if centre > 0: memberships.append([(notMoving+centre)/2, NO_TURN])
		if leftSmall > 0: memberships.append([(notMoving+leftSmall)/2, SOFT_LEFT_TURN])
		if leftBig > 0: memberships.append([(notMoving+leftBig)/2, SOFT_LEFT_TURN])

	if (movingLeftSmall > 0): 
		if rightBig > 0: memberships.append([(movingLeftSmall+rightBig)/2, SOFT_LEFT_TURN])
		if rightSmall > 0: memberships.append([(movingLeftSmall+rightSmall)/2, NO_TURN])
		if centre > 0: memberships.append([(movingLeftSmall+centre)/2, SOFT_LEFT_TURN])
		if leftSmall > 0: memberships.append([(movingLeftSmall+leftSmall)/2, SOFT_LEFT_TURN])
		if leftBig > 0: memberships.append([(movingLeftSmall+leftBig)/2, HARD_LEFT_TURN])

	if (movingLeftBig > 0): 
		if rightBig > 0: memberships.append([(movingLeftBig+rightBig)/2, SOFT_LEFT_TURN])
		if rightSmall > 0: memberships.append([(movingLeftBig+rightSmall)/2, SOFT_LEFT_TURN])
		if centre > 0: memberships.append([(movingLeftBig+centre)/2, SOFT_LEFT_TURN])
		if leftSmall > 0: memberships.append([(movingLeftBig+leftSmall)/2, HARD_LEFT_TURN])
		if leftBig > 0: memberships.append([(movingLeftBig+leftBig)/2, HARD_LEFT_TURN])

	weightedAreaSum = 0
	areaSum = 0	

	for m in memberships:
		m[0] = 0.5 * m[0] * (1 - (m[0]/2))
		weightedAreaSum += m[0] * m[1]
		areaSum += m[0]

	global turnRatio

	if (areaSum != 0):
		turnRatio += weightedAreaSum / areaSum

	if turnRatio > 1: turnRatio = 1
	elif turnRatio < -1: turnRatio = -1

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


def objectDataCallback(data):
	global previousObjectPosition	

	objectPosition = data.data[0]
	objectDimension = data.data[1]
	power = data.data[2]

	#Negative for right movement, positive for left
	lateralDeltaV = objectPosition - previousObjectPosition
	print lateralDeltaV

	leftPower, rightPower = adjustPowerForTurning(power, objectPosition, lateralDeltaV)

	#print "Object position: ", objectPosition, " Object movement: ", lateralDeltaV, "Left Power: ", leftPower, "Right Power: ", rightPower
	publishData(leftPower, rightPower)
	previousObjectPosition = objectPosition


def listener():
	rospy.Subscriber('/zumo/raw_power', Int16MultiArray, objectDataCallback)
	rospy.spin()

if __name__ == '__main__':
	listener()
