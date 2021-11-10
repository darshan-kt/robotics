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
from aSTaR import *

class tree:

	def __init__(self,location,orientation,alpha,centre,g,f):
		self.location = location
		self.orientation = orientation
		self.alpha = alpha
		self.centre = centre
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


class Astar_BiCycle:
	def __init__(self,stGo,orientation,img):
		# self.obstacle_pts = obstacle_pts                #List of obstacle locations
		self.stGo = stGo                           
		self.orientation = orientation     #list of start and goal locations
		self.img = img                                  #map for visualizing
		self.img_h,self.img_w = img.shape

		self.len_veh = 5				
		self.velocity = 5				
		self.time_step = 3

		kernel = np.ones((5,5), np.uint8) 
		img_dilate = cv2.dilate(img.copy(), kernel, iterations=1)

		print("A* Simulation")
		obj = Astar(stGo,img_dilate)
		solution,img_ret,self.path_heu_pts = obj.main_class()
		print("Astar : ",solution)

		print("#####################")
		print(self.path_heu_pts)
		cv2.imshow('image',img_ret)
		cv2.waitKey(2000)
		print("#####################")

		self.path_heu_ptsUNdist = self.cost_at_eachPt(self.path_heu_pts)

	def distance(self,pt1,pt2):                             #Euclidean Distance
		return np.sqrt((pt2[0]-pt1[0])*(pt2[0]-pt1[0]) + (pt2[1]-pt1[1])*(pt2[1]-pt1[1]))

	def cost_at_eachPt(self,pts):
		
		pts_und_Dist = []

		dist = 0
		prev = pts[-1]
		i = len(pts) - 1
		while(i >= 0):
			dist = dist + self.distance(prev,pts[i])
			pts_und_Dist.append([pts[i],dist])
			prev = pts[i]
			i = i - 1

		pts_und_Dist.reverse()

		return pts_und_Dist

	def pt_inORout(self,pt,pts):
		if(round(self.distance(pt,pts[0]) + self.distance(pt,pts[1]) - self.distance(pts[0],pts[1])) == 0):
			return True
		else:
			return False

	def perpend_drop(self,pt,pts):
		[x1,y1] = pts[0]
		[x2,y2] = pts[1]
		[x3,y3] = pt


	 	k = ((y2-y1) * (x3-x1) - (x2-x1) * (y3-y1))/((y2-y1)**2 + (x2-x1)**2)
		x4 = x3 - k * (y2-y1)
		y4 = y3 + k * (x2-x1)

		return [x4,y4], self.distance([x4,y4],pt), self.pt_inORout([x4,y4],pts)

	def finNeaPt(self,pt,pts):

		dist = float("inf")
		i = 0
		j = -1
		while(i < len(pts)):
			if(self.distance(pt,pts[i]) < dist):
				dist = self.distance(pt,pts[i])
				j = i
			i = i + 1

		return j

	def finHeuPt(self,pt):

		pts = self.path_heu_pts
		ind_clsPt = self.finNeaPt(pt, pts)
		if(ind_clsPt < len(pts)-1):
			pt_drop,dist_drop,bool_drop = self.perpend_drop(pt,[pts[ind_clsPt],pts[ind_clsPt+1]])

			if(bool_drop == True):
				return ind_clsPt+1
			else:
				return ind_clsPt
		else:
			return ind_clsPt

	def Heuristic(self,pt):

		HeuPt_ind = self.finHeuPt(pt)

		return self.distance(pt,self.path_heu_pts[HeuPt_ind]) + self.path_heu_ptsUNdist[HeuPt_ind][1]

	def create_actionSet(self,node,orientation):
		len_veh = self.len_veh
		velocity = self.velocity
		time_step = self.time_step

		loc = node	
		actSet = []
		alpha = -np.pi/6

		while(alpha <= np.pi/6):
			if(round(alpha*180/np.pi) != 0):
				radi = len_veh/math.tan(alpha)
				orientation_new = orientation + (velocity*math.tan(alpha)/len_veh)*(time_step)
				loc_new = [loc[0] + (len_veh/math.tan(alpha))*(math.sin(orientation_new) - math.sin(orientation)), loc[1] + (len_veh/math.tan(alpha))*(math.cos(orientation) - math.cos(orientation_new))]

				centre_botFram = [0,radi]
				theta_trans = -orientation
				loc_trans = [-loc[0],-loc[1]]
				centre_global = [radi*math.sin(theta_trans) - loc_trans[0], radi*math.cos(theta_trans) - loc_trans[1]]

			else:
				loc_new = [loc[0] + velocity*math.cos(orientation)*time_step,loc[1] + velocity*math.sin(orientation)*time_step]
				orientation_new = orientation
				centre_global = [np.nan,np.nan]

			actSet.append([loc_new,time_step,orientation_new,alpha,centre_global])
			alpha = alpha + np.pi/180

		return actSet


	def disp_action(self,img,loc1,loc2,alpha,centre):
		len_veh = self.len_veh
		velocity = self.velocity
		time_step = self.time_step

		if(round(alpha*180/np.pi) != 0):
			radi = len_veh/math.tan(alpha)
			time_rev = 2*np.pi*abs(radi)/velocity

			if(time_step < time_rev):
				ang1 = math.atan2(loc1[1] - centre[1],loc1[0] - centre[0])
				ang2 = math.atan2(loc2[1] - centre[1],loc2[0] - centre[0]) 

				if(math.tan(alpha) > 0):
					if(ang2 < ang1):
						ang2 = ang2 + 2*np.pi
				else:
					if(ang2 > ang1):
						ang1 = ang1 + 2*np.pi
				cv2.ellipse(img,(int(centre[0]),int(centre[1])),(int(abs(radi)),int(abs(radi))),0,ang1*180/np.pi,ang2*180/np.pi,255,1)

			else:
				cv2.circle(img,(int(centre[0]),int(centre[1])),int(abs(radi)),255,1)

			cv2.circle(img,(int(loc1[0]),int(loc1[1])),2,155,-1)
			cv2.circle(img,(int(loc2[0]),int(loc2[1])),2,155,-1)

		else:
			cv2.line(img,(int(loc1[0]),int(loc1[1])),(int(loc2[0]),int(loc2[1])),255,1)
			cv2.circle(img,(int(loc2[0]),int(loc2[1])),2,155,-1)


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

		start_node = tree(start,self.orientation,0,[np.nan,np.nan],0,self.Heuristic(start))

		que_open.push(start_node)                       #que_open is created for popping an element in each iteration 
		dict_open[start_node] = start_node              #dict_open is created for checking whether an element is in the OPEN list.
		time1 = time.time()

		i = 0
		while(que_open.leng() > 0):
			node_current = que_open.pop()           #popping element
			# if(i % 10000 == 0):
			cv2.imshow('image',img)
			cv2.waitKey(100)

			del(dict_open[node_current])            #deleting it from the dictionary
			loc = node_current.location

			if(self.distance(node_current.location,goal) < 20):     #breaks the loop if it reaches goal
				solution = True
				break

			closed.append(node_current)                             #adding the popped element to closed 
			dict_closed[node_current] = node_current                #adding the popped element to closed dictionary
			action_set = self.create_actionSet(node_current.location,node_current.orientation)         #creating action set

			for action in action_set:                               #running the loop for creating new children

				node_successor_loc = action[0]           
 				node_successor_g = node_current.g + action[1]
				node_successor_f = node_successor_g + self.Heuristic(node_successor_loc)

				node_successor_orien = action[2]

				node_successor = tree(node_successor_loc,node_successor_orien,action[3],action[4],node_successor_g,node_successor_f)

				node_successor.parent = node_current
			
				val_open = node_successor in dict_open
				val_closed = node_successor in dict_closed
				val0 = self.obstacle_check(node_successor_loc)

				if(val0 == False):
					self.disp_action(img,node_current.location,action[0],action[3],action[4])

					if(val_closed == False):

						if(val_open == False):
							que_open.push(node_successor)   #pushing the chldi node to OPEN list if it is not in OPEN or CLOSED list
							dict_open[node_successor] = node_successor

						else:                                   #The child node is not in CLOSED list but it is in OPEN list
							open_successor = dict_open[node_successor]

							if(node_successor.g < open_successor.g):        #if current child node g value is less than previously created node g value
								open_successor.g = node_successor.g
								open_successor.f = node_successor.f
								open_successor.alpha = node_successor.alpha
								open_successor.centre = node_successor.centre
								open_successor.parent = node_successor.parent
								open_successor.orientation = node_successor.orientation
			i = i + 1

		time2 = time.time()

		prev = node_current
		dts_path = []
		img2 = self.img.copy()

		while(solution == True):
			cur = prev.parent
			dts_path.append([cur.location,prev.location,prev.alpha,prev.centre])

			if(round(cur.location[0] - start[0]) == 0 and round(cur.location[1] - start[1]) == 0):
				break

			prev = cur

		return solution,img,dts_path


img = cv2.imread('MaInmAP.png',0)
# img = cv2.imread('MaInmAP_mINi.png',0)
# img = cv2.imread('MaInmAP_mINi2.png',0)
# img = cv2.imread('tEsTmAP.png',0)

inf_stgo = [[90,465],[470,460]]
obj = Astar_BiCycle(inf_stgo,-np.pi,img)
solution,img_ret,path_pts = obj.main_class()

for pt in path_pts:
	# print(pt)
	obj.disp_action(img,pt[0],pt[1],pt[2],pt[3])

cv2.imshow('image',img)
cv2.waitKey(0)

# actionset = obj.create_actionSet(loc,theta1)
# obj.disp_action(img,loc,actionset[i][0],actionset[i][3],actionset[i][4])




