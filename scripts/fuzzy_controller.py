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

def publishData(leftPower, rightPower):
	data = Int16MultiArray()
	data.data = [leftPower, rightPower]
	pub.publish(data)

def adjustPowerForTurning(power, ballPosition):	
	#Ball position of 300 is roughly centre of FOV.
	#Tracks turn off at mid values, and move in opposite direction at max values
	LEFT_TURN_MAX = 30
	LEFT_TURN_MID = 165
	NO_TURN = 300
	RIGHT_TURN_MAX = 570
	RIGHT_TURN_MID = 435

#	if ballPosition <= LEFT_TURN_MAX:
#		if power >= 0:
#			return -power, power
#		return power, -power
#	
# 
#	if ballPosition < LEFT_TURN_MID:
#		if ballPosition < 31: ballPosition = 31
#		normalisedRatio = (ballPosition-LEFT_TURN_MAX) / (LEFT_TURN_MID-LEFT_TURN_MAX)
#		if power >= 0:
#			return -power*(1-normalisedRatio), power
#		return power, -power*(1-normalisedRatio)
	if ballPosition < NO_TURN:
		if ballPosition < LEFT_TURN_MAX: ballPosition = LEFT_TURN_MAX
		normalisedRatio = (ballPosition-LEFT_TURN_MAX) / (NO_TURN-LEFT_TURN_MAX)
		if power >= 0:
			return power*normalisedRatio, power
		return power, power*normalisedRatio
#	if ballPosition >= RIGHT_TURN_MAX:
#		if power >= 0:
#			return power, -power
#		return -power, power
#	if ballPosition >= RIGHT_TURN_MID:
#		if ballPosition > 569: ballPosition = 569
#		normalisedRatio = (ballPosition-RIGHT_TURN_MID) / (RIGHT_TURN_MAX-RIGHT_TURN_MID)
#		if power >= 0:
#			return power, -power*(normalisedRatio)
#		return -power*(normalisedRatio), power
	if ballPosition > NO_TURN:
		if ballPosition > RIGHT_TURN_MAX: ballPosition = RIGHT_TURN_MAX
		normalisedRatio = (ballPosition-NO_TURN) / (RIGHT_TURN_MAX-NO_TURN)
		if power >= 0:
			return power, power*(1-normalisedRatio)
		return power*(1-normalisedRatio), power	
	return power, power

rospy.init_node('fuzzy_controller', anonymous=True)
pub = rospy.Publisher('/zumo/power', Int16MultiArray, queue_size = 10)

#Size of target
targetUndersizeBig = FuzzyTrapezoid(0, 0, 60, 70)
targetUndersizeSmall = FuzzyTriangle(65, 80, 88)
targetIdealSize = FuzzyTriangle(85, 90, 100)
targetOversizeSmall = FuzzyTriangle(92, 105, 120)
targetOversizeBig = FuzzyTrapezoid(115, 140, 500, 500)

#Rate of change in relation to target between readings
deceleratingBig = FuzzyTrapezoid(-20, -20, -10, -5)
deceleratingSmall = FuzzyTriangle(-6, -3, 0)
speedConstant = FuzzyTriangle(-5, 0, 5)
acceleratingSmall = FuzzyTriangle(0, 3, 6)
acceleratingBig = FuzzyTrapezoid(2.5, 10, 20, 20)

BIG_SPEED_CHANGE = 40
SMALL_SPEED_CHANGE = 20
NO_SPEED_CHANGE = 0

POWER_LIMIT = 120
power = 0
previousBallDimension = 0

def callback(data):
	global power
	global previousBallDimension
	ballPosition = data.data[0]
	ballDimension = data.data[1]
	#Change in velocity relative to target
	deltaV = ballDimension - previousBallDimension
	
	closeBig = targetOversizeBig.getMembership(ballDimension)
	closeSmall = targetOversizeSmall.getMembership(ballDimension)
	atTarget = targetIdealSize.getMembership(ballDimension)
	farSmall = targetUndersizeSmall.getMembership(ballDimension)
	farBig = targetUndersizeBig.getMembership(ballDimension)

	expandingBig = deceleratingBig.getMembership(deltaV)
	expandingSmall = deceleratingSmall.getMembership(deltaV)
	constantSpeed = speedConstant.getMembership(deltaV)
	closingSmall = acceleratingSmall.getMembership(deltaV)
	closingBig = acceleratingBig.getMembership(deltaV)

	print "closeBig:   ", closeBig
	print "closeSmall: ", closeSmall
	print "atTarget:   ", atTarget
	print "farSmall:   ", farSmall
	print "farBig:     ", farBig
	print ""
	print "expandingBig:   ", expandingBig
	print "expandingSmall: ", expandingSmall
	print "constantSpeed:  ", constantSpeed
	print "closingSmall:   ", closingSmall
	print "closingBig:     ", closingBig

	memberships = []
	if closeBig > 0:		
		if expandingBig > 0: memberships.append([(closeBig+expandingBig)/2, NO_SPEED_CHANGE])
		if expandingSmall > 0: memberships.append([(closeBig+expandingSmall)/2, -BIG_SPEED_CHANGE])
		if constantSpeed > 0: memberships.append([(closeBig+constantSpeed)/2, -BIG_SPEED_CHANGE])
		if closingSmall > 0: memberships.append([(closeBig+closingSmall)/2, -BIG_SPEED_CHANGE])
		if closingBig > 0: memberships.append([(closeBig+closingBig)/2, -BIG_SPEED_CHANGE])
		
	if (closeSmall > 0):
		#Changed from SMALL_SPEED_CHANGE
		if expandingBig > 0: memberships.append([(closeSmall+expandingBig)/2, BIG_SPEED_CHANGE])
		#Changed from NO_SPEED_CHANGE
		if expandingSmall > 0: memberships.append([(closeSmall+expandingSmall)/2, SMALL_SPEED_CHANGE])
		if constantSpeed > 0: memberships.append([(closeSmall+constantSpeed)/2, -SMALL_SPEED_CHANGE])
		if closingSmall > 0: memberships.append([(closeSmall+closingSmall)/2, -SMALL_SPEED_CHANGE])
		if closingBig > 0: memberships.append([(closeSmall+closingBig)/2, -BIG_SPEED_CHANGE])

	if (atTarget > 0): 
		if expandingBig > 0: memberships.append([(atTarget+expandingBig)/2, BIG_SPEED_CHANGE])
		if expandingSmall > 0: memberships.append([(atTarget+expandingSmall)/2, SMALL_SPEED_CHANGE])
		if constantSpeed > 0: memberships.append([(atTarget+constantSpeed)/2, NO_SPEED_CHANGE])
		if closingSmall > 0: memberships.append([(atTarget+closingSmall)/2, -SMALL_SPEED_CHANGE])
		if closingBig > 0: memberships.append([(atTarget+closingBig)/2, -BIG_SPEED_CHANGE])

	if (farSmall > 0): 
		if expandingBig > 0: memberships.append([(farSmall+expandingBig)/2, BIG_SPEED_CHANGE])
		if expandingSmall > 0: memberships.append([(farSmall+expandingSmall)/2, SMALL_SPEED_CHANGE])
		if constantSpeed > 0: memberships.append([(farSmall+constantSpeed)/2, SMALL_SPEED_CHANGE])
		#Changed from NO_SPEED_CHANGE
		if closingSmall > 0: memberships.append([(farSmall+closingSmall)/2, -SMALL_SPEED_CHANGE])
		if closingBig > 0: memberships.append([(farSmall+closingBig)/2, -BIG_SPEED_CHANGE])

	if (farBig > 0): 
		if expandingBig > 0: memberships.append([(farBig+expandingBig)/2, BIG_SPEED_CHANGE])
		if expandingSmall > 0: memberships.append([(farBig+expandingSmall)/2, BIG_SPEED_CHANGE])
		if constantSpeed > 0: memberships.append([(farBig+constantSpeed)/2, BIG_SPEED_CHANGE])
		if closingSmall > 0: memberships.append([(farBig+closingSmall)/2, BIG_SPEED_CHANGE])
		if closingBig > 0: memberships.append([(farBig+closingBig)/2, NO_SPEED_CHANGE])

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

	leftPower, rightPower = adjustPowerForTurning(power, ballPosition)
	print "Ball position: ", ballPosition, " Ball dimension: ", ballDimension, "Left Power: ", leftPower, "Right Power: ", rightPower
	publishData(leftPower, rightPower)
	previousBallDimension = ballDimension

def listener():
	rospy.Subscriber('/zumo/ball_pos', Int16MultiArray, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
