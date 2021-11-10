import numpy as np
import math


class tree:
	def __init__(self,location,orientation,alpha,centre,g,f):
		self.location = location
		self.orientation = orientation
		self.alpha = alpha
		self.centre = centre
		self.parent = None
		self.g = g
		self.f = f

	def __hash__(self):
		return hash((round(self.location[0]),round(self.location[1]),round(self.orientation*180/np.pi)))

	def __eq__(self,other):
		return (round(self.location[0]),round(self.location[1]),round(self.orientation*180/np.pi)) == (round(other.location[0]),round(other.location[1]),round(other.orientation*180/np.pi))

	def __le__(self,other):
		return self.f <= other.f

	def __lt__(self,other):
		return self.f < other.f
