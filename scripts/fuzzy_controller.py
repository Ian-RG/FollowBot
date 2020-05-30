#!/usr/bin/env python
from __future__ import division
from time import sleep
import rospy
from std_msgs.msg import Int16MultiArray

class FuzzyTriangle:
	def __init__(self, left, middle, right):
		self.left = left
		self.middle = middle
		self.right = right
	
	def getMembership(self, v):
		if v < self.left: 
			return 0
		elif v < self.middle: 
			return (v-self.left) / (self.middle-self.left)
		elif v == self.middle: 
			return 1
		elif v < self.right: 
			return (self.right-v) / (self.right-self.middle)
		return 0

class FuzzyTrapezoid:
	def __init__(self, left, midLeft, midRight, right):
		self.left = left
		self.midLeft = midLeft
		self.midRight = midRight
		self.right = right

	def getMembership(self, v):
		if v < self.left: return 0
		elif v < self.midLeft: return (v-self.left) / (self.midLeft-self.left)
		elif v >= self.midLeft and v <= self.midRight: return 1
		elif v < self.right: return (self.right-v) / (self.right-self.midRight)
		return 0

rospy.init_node('fuzzy_controller', anonymous=True)
pub = rospy.Publisher('/zumo/power', Int16MultiArray, queue_size = 10)

#Size of target
targetUndersizeBigFn = FuzzyTrapezoid(0, 0, 60, 70)
targetUndersizeSmallFn = FuzzyTriangle(65, 80, 88)
targetIdealSizeFn = FuzzyTriangle(85, 90, 100)
targetOversizeSmallFn = FuzzyTriangle(92, 105, 120)
targetOversizeBigFn = FuzzyTrapezoid(115, 140, 500, 500)

#Rate of change in relation to target between readings
deceleratingBigFn = FuzzyTrapezoid(-20, -20, -10, -5)
deceleratingSmallFn = FuzzyTriangle(-6, -3, 0)
speedConstantFn = FuzzyTriangle(-5, 0, 5)
acceleratingSmallFn = FuzzyTriangle(0, 3, 6)
acceleratingBigFn = FuzzyTrapezoid(2.5, 10, 20, 20)

#Ball position relative to centre of frame
leftBigFn = FuzzyTrapezoid(-100, -100, 30, 150)
leftSmallFn = FuzzyTriangle(130, 210, 290)
centreFn = FuzzyTriangle(280, 300, 320)
rightSmallFn = FuzzyTriangle(310, 390, 470)
rightBigFn = FuzzyTrapezoid(450, 570, 1000, 1000)

BIG_SPEED_CHANGE = 40
SMALL_SPEED_CHANGE = 20
NO_SPEED_CHANGE = 0
HARD_TURN_LEFT = -1
SOFT_TURN_LEFT = -0.5
NO_TURN = 0
SOFT_TURN_RIGHT = 0.5
HARD_TURN_RIGHT = 1
POWER_LIMIT = 120

power = 0
previousBallDimension = 0



def publishData(leftPower, rightPower):
	data = Int16MultiArray()
	data.data = [leftPower, rightPower]
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
	#print "deceleratingBig:   ", expandingBig
	#print "deceleratingSmall: ", expandingSmall
	#print "speedConstant:  ", constantSpeed
	#print "acceleratingSmall:   ", closingSmall
	#print "acceleratingBig:     ", closingBig

	memberships = []
	if closeBig > 0:		
		if deceleratingBig > 0: memberships.append([(closeBig+deceleratingBig)/2, NO_SPEED_CHANGE])
		if deceleratingSmall > 0: memberships.append([(closeBig+deceleratingSmall)/2, -BIG_SPEED_CHANGE])
		if speedConstant > 0: memberships.append([(closeBig+speedConstant)/2, -BIG_SPEED_CHANGE])
		if acceleratingSmall > 0: memberships.append([(closeBig+acceleratingSmall)/2, -BIG_SPEED_CHANGE])
		if acceleratingBig > 0: memberships.append([(closeBig+acceleratingBig)/2, -BIG_SPEED_CHANGE])
		
	if (closeSmall > 0):
		#Changed from SMALL_SPEED_CHANGE
		if deceleratingBig > 0: memberships.append([(closeSmall+deceleratingBig)/2, BIG_SPEED_CHANGE])
		#Changed from NO_SPEED_CHANGE
		if deceleratingSmall > 0: memberships.append([(closeSmall+deceleratingSmall)/2, SMALL_SPEED_CHANGE])
		if speedConstant > 0: memberships.append([(closeSmall+speedConstant)/2, -SMALL_SPEED_CHANGE])
		if acceleratingSmall > 0: memberships.append([(closeSmall+acceleratingSmall)/2, -SMALL_SPEED_CHANGE])
		if acceleratingBig > 0: memberships.append([(closeSmall+acceleratingBig)/2, -BIG_SPEED_CHANGE])

	if (atTarget > 0): 
		if deceleratingBig > 0: memberships.append([(atTarget+deceleratingBig)/2, BIG_SPEED_CHANGE])
		if deceleratingSmall > 0: memberships.append([(atTarget+deceleratingSmall)/2, SMALL_SPEED_CHANGE])
		if speedConstant > 0: memberships.append([(atTarget+speedConstant)/2, NO_SPEED_CHANGE])
		if acceleratingSmall > 0: memberships.append([(atTarget+acceleratingSmall)/2, -SMALL_SPEED_CHANGE])
		if acceleratingBig > 0: memberships.append([(atTarget+acceleratingBig)/2, -BIG_SPEED_CHANGE])

	if (farSmall > 0): 
		if deceleratingBig > 0: memberships.append([(farSmall+deceleratingBig)/2, BIG_SPEED_CHANGE])
		if deceleratingSmall > 0: memberships.append([(farSmall+deceleratingSmall)/2, SMALL_SPEED_CHANGE])
		if speedConstant > 0: memberships.append([(farSmall+speedConstant)/2, SMALL_SPEED_CHANGE])
		#Changed from NO_SPEED_CHANGE
		if acceleratingSmall > 0: memberships.append([(farSmall+acceleratingSmall)/2, -SMALL_SPEED_CHANGE])
		if acceleratingBig > 0: memberships.append([(farSmall+acceleratingBig)/2, -BIG_SPEED_CHANGE])

	if (farBig > 0): 
		if deceleratingBig > 0: memberships.append([(farBig+deceleratingBig)/2, BIG_SPEED_CHANGE])
		if deceleratingSmall > 0: memberships.append([(farBig+deceleratingSmall)/2, BIG_SPEED_CHANGE])
		if speedConstant > 0: memberships.append([(farBig+speedConstant)/2, BIG_SPEED_CHANGE])
		if acceleratingSmall > 0: memberships.append([(farBig+acceleratingSmall)/2, BIG_SPEED_CHANGE])
		if acceleratingBig > 0: memberships.append([(farBig+acceleratingBig)/2, NO_SPEED_CHANGE])

	weightedAreaSum = 0
	areaSum = 0

	for m in memberships:
		m[0] = 40 * m[0] * (1 - (m[0]/2))
		weightedAreaSum += m[0] * m[1]
		areaSum += m[0]

	if (areaSum != 0):
		powerChange = weightedAreaSum / areaSum
		power += powerChange
		

	if (power > POWER_LIMIT):
		power = POWER_LIMIT
	if (power < -POWER_LIMIT):
		power = -POWER_LIMIT	
	return power

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
		if rightBig > 0: memberships.append([(closeBig+rightBig)/2, HARD_TURN_RIGHT])
		if rightSmall > 0: memberships.append([(closeBig+rightSmall)/2, SOFT_TURN_RIGHT])
		if centre > 0: memberships.append([(closeBig+centre)/2, NO_TURN])
		if leftSmall > 0: memberships.append([(closeBig+leftSmall)/2, SOFT_TURN_LEFT])
		if leftBig > 0: memberships.append([(closeBig+leftBig)/2, SOFT_TURN_LEFT])
		
	if (closeSmall > 0):
		if rightBig > 0: memberships.append([(closeSmall+rightBig)/2, SOFT_TURN_RIGHT])
		if rightSmall > 0: memberships.append([(closeSmall+rightSmall)/2, SOFT_TURN_RIGHT])
		if centre > 0: memberships.append([(closeSmall+centre)/2, NO_TURN])
		if leftSmall > 0: memberships.append([(closeSmall+leftSmall)/2, SOFT_TURN_LEFT])
		if leftBig > 0: memberships.append([(closeSmall+leftBig)/2, SOFT_TURN_LEFT])

	if (atTarget > 0): 
		if rightBig > 0: memberships.append([(atTarget+rightBig)/2, SOFT_TURN_RIGHT])
		if rightSmall > 0: memberships.append([(atTarget+rightSmall)/2, SOFT_TURN_RIGHT])
		if centre > 0: memberships.append([(atTarget+centre)/2, NO_TURN])
		if leftSmall > 0: memberships.append([(atTarget+leftSmall)/2, SOFT_TURN_LEFT])
		if leftBig > 0: memberships.append([(atTarget+leftBig)/2, SOFT_TURN_LEFT])

	if (farSmall > 0): 
		if rightBig > 0: memberships.append([(farSmall+rightBig)/2, SOFT_TURN_RIGHT])
		if rightSmall > 0: memberships.append([(farSmall+rightSmall)/2, NO_TURN])
		if centre > 0: memberships.append([(farSmall+centre)/2, NO_TURN])
		if leftSmall > 0: memberships.append([(farSmall+leftSmall)/2, NO_TURN])
		if leftBig > 0: memberships.append([(farSmall+leftBig)/2, SOFT_TURN_LEFT])

	if (farBig > 0): 
		if rightBig > 0: memberships.append([(farBig+rightBig)/2, SOFT_TURN_RIGHT])
		if rightSmall > 0: memberships.append([(farBig+rightSmall)/2, NO_TURN])
		if centre > 0: memberships.append([(farBig+centre)/2, NO_TURN])
		if leftSmall > 0: memberships.append([(farBig+leftSmall)/2, NO_TURN])
		if leftBig > 0: memberships.append([(farBig+leftBig)/2, SOFT_TURN_LEFT])

	weightedAreaSum = 0
	areaSum = 0

	turnRatio = 0

	for m in memberships:
		m[0] = 40 * m[0] * (1 - (m[0]/2))
		weightedAreaSum += m[0] * m[1]
		areaSum += m[0]

	if (areaSum != 0):
		turnRatio = weightedAreaSum / areaSum

	#When reversing, the track opposite the turn needs to be slowed (ie slow the right track for a left turn when reversing)
	leftPower = rightPower = power
	if (turnRatio < 0):
		if (power > 0): leftPower *= (-turnRatio)
		else: rightPower *= (-turnRatio)
	elif (turnRatio > 0):
		if (power > 0): rightPower *= turnRatio
		else: leftPower *= turnRatio
	
	print turnRatio
		
	return leftPower, rightPower


def callback(data):
	global power
	global previousBallDimension
	ballPosition = data.data[0]
	ballDimension = data.data[1]
	#Change in velocity relative to target
	deltaV = ballDimension - previousBallDimension

	power = getPower(power, ballDimension, deltaV)
	leftPower, rightPower = adjustPowerForTurning(power, ballPosition, ballDimension)

	#print "Ball position: ", ballPosition, " Ball dimension: ", ballDimension, "Left Power: ", leftPower, "Right Power: ", rightPower
	publishData(leftPower, rightPower)
	previousBallDimension = ballDimension

def listener():
	rospy.Subscriber('/zumo/ball_pos', Int16MultiArray, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
