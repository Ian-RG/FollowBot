from __future__ import division

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
