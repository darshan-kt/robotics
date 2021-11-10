import cv2
import numpy as np 
import math
import time
import ast 
import csv
from Queue import PriorityQueue
from shapely.geometry import Polygon,Point
from scipy.interpolate import interp1d
from scipy import interpolate



# Global Planner : A* with -90,0,+90 actions and Collision Checking is done by checking the location of node in the map.
# The orientation is included for checking the nodes in the Open and Closed list.

class tree:
	def __init__(self,location,orientation,g,f):
		self.location = location
		self.orientation = orientation
		self.parent = None
		self.g = g
		self.f = f

#hash value for the tree object
#We are dividing x,y with win_len and flooring that value, it means we are creating only one node for each grid (win_len*win_len size)
	def __hash__(self):
		return hash((math.floor(self.location[0]),math.floor(self.location[1]),self.orientation))

#To compare two tree objects, operator(=)
	def __eq__(self,other):
		return (math.floor(self.location[0]),math.floor(self.location[1]),self.orientation) == (math.floor(other.location[0]),math.floor(other.location[1]),other.orientation)

#To compare two tree objects, operator(<=)
	def __le__(self,other):
		self_val = self.f
		other_val = other.f
		return self_val <= other_val
#To compare two tree objects, operator(<)
	def __lt__(self,other):
		self_val = self.f
		other_val = other.f
		return self_val < other_val

#Priority Queue
class Queue:
	def __init__(self):
		self.list_queue = PriorityQueue()

	def push(self,node):
		self.list_queue.put(node)

	def distance(self,pt1,pt2):
		return np.sqrt((pt2[0]-pt1[0])*(pt2[0]-pt1[0]) + (pt2[1]-pt1[1])*(pt2[1]-pt1[1]))

	def pop(self):
		node = self.list_queue.get()
		return node

	def leng(self):
		return self.list_queue.qsize()

#Returns true if it is not empty
	def isfull(self):
		return np.invert(self.list_queue.empty())


class Astar:
	def __init__(self,stGo,orientation,img):
		# self.obstacle_pts = obstacle_pts                #List of obstacle locations
		self.stGo = stGo                           
		self.orientation = orientation     #list of start and goal locations
		self.img = img                                  #map for visualizing
		self.img_h,self.img_w = img.shape

		self.len_veh = 5					#Length of the vehicle is 5PIXELS
		self.velocity = 10					#Vehicle velocity is 10PIXELS/SEC
		self.tim_step = 2 

	def distance(self,pt1,pt2):                             #Euclidean Distance
		return np.sqrt((pt2[0]-pt1[0])*(pt2[0]-pt1[0]) + (pt2[1]-pt1[1])*(pt2[1]-pt1[1]))

	def pt_shift(self,node,list_nodes):                     #Finding the new location after shifting the axis
		list_ret = []
		i = 0
		while (i < len(list_nodes)):
			list_ret.append([list_nodes[i][0]-node[0],list_nodes[i][1]-node[1]])
			i = i + 1

		return list_ret	

	def create_action(self,node,orientation):               #Creating the action set, each action contains the children node location in global frame,
                                                                #orientation of the children node in global frame, cost of action                                                                #and the location of convex hull in global frame
		angle = orientation*np.pi/180

		pt1 = [20*math.cos(angle+np.pi/2),20*math.sin(angle+np.pi/2)]
		pt2 = [20*math.cos(angle),20*math.sin(angle)]
		pt3 = [20*math.cos(angle-np.pi/2),20*math.sin(angle-np.pi/2)]

		pt1 = self.pt_shift([-node[0],-node[1]],[pt1])
		pt2 = self.pt_shift([-node[0],-node[1]],[pt2])
		pt3 = self.pt_shift([-node[0],-node[1]],[pt3])

		action1 = [pt1,21,orientation + 90]
		action2 = [pt2,20,orientation]
		action3 = [pt3,21,orientation - 90]
		action_set = [action1,action2,action3]

		return action_set

#collision checking starts here
	def obstacle_check(self,node):

		if(node[0] < 0 or node[0] >= self.img_w or node[1] < 0 or node[1] >=self.img_h or self.img[int(node[1])][int(node[0])] > 1):
			return True
		else:
			return False

	def main_class(self):                                   #main

		img = self.img.copy()
		inf_stgo = self.stGo
		cv2.circle(img,tuple(inf_stgo[0]),3,255,-1)
		cv2.circle(img,tuple(inf_stgo[1]),3,255,-1)
		cv2.circle(img,tuple(inf_stgo[1]),20,255,1)
		start = inf_stgo[0]                             #start location
		goal = inf_stgo[1]                              #goal location
		que_open = Queue()                              #OPEN (priority queue)
		closed = []                                     #CLOSED list
		dict_open = {}                                  #OPEN (dictionary), created for fast checking of node in the dictionary. Checking is done by __hash__
		dict_closed = {}        

		solution = False

		start_node = tree(start,self.orientation,0,self.distance(start,goal))
		que_open.push(start_node)                       #que_open is created for popping an element in each iteration 
		dict_open[start_node] = start_node              #dict_open is created for checking whether an element is in the OPEN list.

		while(que_open.leng() > 0):
			node_current = que_open.pop()           #popping element
			del(dict_open[node_current])            #deleting it from the dictionary
			loc = node_current.location
			# cv2.circle(img,(int(loc[0]),int(loc[1])),2,255,-1)      #For drawing circle on the image (animation)

			if(self.distance(node_current.location,goal) < 20):     #breaks the loop if it reaches goal
				solution = True
				break

			closed.append(node_current)                             #adding the popped element to closed 
			dict_closed[node_current] = node_current                #adding the popped element to closed dictionary
			action_set = self.create_action(node_current.location,node_current.orientation)         #creating action set

			for action in action_set:                               #running the loop for creating new children
				# cv2.imshow('image_A*',img)
				# cv2.waitKey(1)

				node_successor_loc = action[0][0]           
				node_successor_g = node_current.g + action[1]
				node_successor_f = node_successor_g + self.distance(node_successor_loc,goal)

				node_successor_orien = action[2]
				node_successor = tree(node_successor_loc,node_successor_orien,node_successor_g,node_successor_f)
				node_successor.parent = node_current
			
				val_open = node_successor in dict_open
				val_closed = node_successor in dict_closed
				val0 = self.obstacle_check(node_successor_loc)
						
				if(val0 == False):
					if(val_closed == False):
						if(val_open == False):
							que_open.push(node_successor)   #pushing the chldi node to OPEN list if it is not in OPEN or CLOSED list
							dict_open[node_successor] = node_successor
							cv2.circle(img,tuple(np.asarray(node_successor.location,np.int)),2,255,-1)
							cv2.line(img,tuple(np.asarray(node_successor.location,np.int)),tuple(np.asarray(node_current.location,np.int)),255,1)
						else:                                   #The child node is not in CLOSED list but it is in OPEN list
							open_successor = dict_open[node_successor]
							if(node_successor.g < open_successor.g):        #if current child node g value is less than previously created node g value
								open_successor.g = node_successor.g
								open_successor.parent = node_successor.parent
								open_successor.f = node_successor.f
								open_successor.orientation = node_successor.orientation

		prev = goal
		pts_path = []
		while(solution == True):
			cur = node_current.location
			pts_path.append(cur)
			cv2.circle(img,(int(cur[0]),int(cur[1])),2,155,-1)      #drawing circle at current location(the node which is in the circle of acceptance)
			cv2.line(img,(int(cur[0]),int(cur[1])),(int(prev[0]),int(prev[1])),155,3)
			if(node_current.location[0] == start[0] and node_current.location[1] == start[1]):
				cv2.circle(img,(int(start[0]),int(start[1])),5,155,-1)
				cv2.circle(img,(int(goal[0]),int(goal[1])),5,155,-1)
				break
			node_current = node_current.parent

			prev = cur

		return solution,img,pts_path


# img = cv2.imread('tEsTmAP.png',0)
img = cv2.imread('MaInmAP.png',0)
inf_stgo = [[90,465],[470,460]]
obj = Astar(inf_stgo,-90,img)
solution,img_ret,path_pts = obj.main_class()
print(path_pts)

while(True):
	cv2.imshow('image_A*',img_ret)
	if(cv2.waitKey(1)&0xFF == 27):
		break
