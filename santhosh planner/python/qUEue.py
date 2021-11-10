import numpy as np 
from Queue import PriorityQueue

class queue:
	def __init__(self):
		self.list_queue = PriorityQueue()

	def push(self,node):
		self.list_queue.put(node)
	
	def distance(self,pt1,pt2):
		return np.sqrt((pt2[0]-pt1[0])*(pt2[0]-pt1[0]) + (pt2[1]-pt1[1])*(pt2[1]-pt1[1]))

	def pop(self):
		node = self.list_queue.get()
		return node

	def leng(self):
		return self.list_queue.qsize()

	def isfull(self):
		return np.invert(self.list_queue.empty())

