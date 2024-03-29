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

	def __hash__(self):
		return hash((math.floor(self.location[0]),math.floor(self.location[1]),self.orientation))

	def __eq__(self,other):
		return (math.floor(self.location[0]),math.floor(self.location[1]),self.orientation) == (math.floor(other.location[0]),math.floor(other.location[1]),other.orientation)

	def __le__(self,other):
		return self.f <= other.f

	def __lt__(self,other):
		return self.f < other.f

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

	def isfull(self):
		return np.invert(self.list_queue.empty())

class grassFire:
	def __init__(self,img,loc_goal,neighHoodSize):
		self.img = img
		self.loc_goal = loc_goal
		self.neighHoodSize = neighHoodSize

	def distance(self,pt1,pt2):
		return math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)

	def inRo(self,val):
		return int(round(val))

	def obstacle_check(self,node1,node2):
		img_h = self.img.shape[0]
		img_w = self.img.shape[1]	

		if(node2[0] < 0 or node2[0] >= img_w or node2[1] < 0 or node2[1] >= img_h):
			return True

		else:
			x_list = [node1[0],node2[0]]
			y_list = [node1[1],node2[1]]

			x_mi = self.inRo(np.amin(x_list)) - 3
			x_ma = self.inRo(np.amax(x_list)) + 3
			y_mi = self.inRo(np.amin(y_list)) - 3
			y_ma = self.inRo(np.amax(y_list)) + 3

			if(x_mi < 0):
				x_mi = 0
			if(x_ma >= img_w):
				x_ma = img_w - 1
			if(y_mi < 0):
				y_mi = 0
			if(y_ma >= img_h):
				y_ma = img_h - 1

			img_obst_roi = self.img[y_mi:y_ma,x_mi:x_ma]
			origin_roi = [x_mi,y_mi]
			node1_new = [self.inRo(node1[0] - origin_roi[0]),self.inRo(node1[1] - origin_roi[1])]
			node2_new = [self.inRo(node2[0] - origin_roi[0]),self.inRo(node2[1] - origin_roi[1])]

			win_action = np.zeros(img_obst_roi.shape,np.uint8)
			cv2.line(win_action,tuple(node1_new),tuple(node2_new),255,1)

			obs_coll = cv2.bitwise_and(win_action,img_obst_roi)
			if(np.amax(obs_coll) > 10):
				return True
			else:
				return False

	def genNeighb(self,pt,matShape):
		x_low = 0
		x_high = matShape[1]
		y_low = 0
		y_high = matShape[0]
		neighHoodSize = self.neighHoodSize

		neighGlob = []

		i = -1*neighHoodSize
		while(i <= neighHoodSize):

			j = -1*neighHoodSize
			while(j <= neighHoodSize):
				if(i == 0 and j == 0):
					pass
			
				else:
					neighPt = [pt[0] + i,pt[1] + j]

					if(neighPt[0] >= 0 and neighPt[0] < x_high and neighPt[1] >= 0 and neighPt[1] < y_high):
						neighGlob.append([neighPt,self.distance(pt,neighPt)])

				j = j + 1
			i = i + 1

		return neighGlob

	def genCosts(self):
		img = self.img.copy()
		img_disp = img.copy()
		loc_goal = self.loc_goal[:]

		mat_costs = np.dot(np.ones(img.shape),np.inf)		
		mat_costs[loc_goal[1]][loc_goal[0]] = 0
		list_open = [loc_goal]

		while(len(list_open) != 0):
			curr = list_open[0]
			del list_open[0]

			childNodes = self.genNeighb(curr,img.shape)

			j = 0
			while(j < len(childNodes)):
				child = childNodes[j]
				if(img[child[0][1]][child[0][0]] == 0):
					if(mat_costs[curr[1]][curr[0]] + child[1] < mat_costs[child[0][1]][child[0][0]]):
						mat_costs[child[0][1]][child[0][0]] = mat_costs[curr[1]][curr[0]] + child[1]
						list_open.append(child[0])

				j = j + 1

		return mat_costs 

class Astar_BiCycle:
	def __init__(self,stGo,orientation,img):
		self.stGo = stGo
		self.orientation = orientation     #list of start and goal locations
		self.img = img                                  #map for visualizing
		self.img_h,self.img_w = img.shape

		self.len_veh = 5
		self.velocity = 5
		self.time_step = 3

		# kernel = np.ones((5,5), np.uint8) 
		# img_dilate = cv2.dilate(img.copy(), kernel, iterations=1)
		# grassObj = grassFire(img_dilate,stGo[1],1)

		grassObj = grassFire(img,stGo[1],1)
		self.costs_heuristic = grassObj.genCosts()
		print("Heuristic Generated")


		# print("A* Simulation")
		# obj = Astar(stGo,img)
		# # obj = Astar(stGo,img_dilate)
		# solution,self.img_Heu,self.path_heu_pts = obj.main_class()

		# print("Astar : " + str(solution))
		# print('###############')

		# cv2.imshow('image',self.img_Heu)
		# cv2.waitKey(0)

		# self.path_heu_ptsUNdist = self.cost_at_eachPt(self.path_heu_pts)

	def distance(self,pt1,pt2):                             #Euclidean Distance
		return np.sqrt((pt2[0]-pt1[0])*(pt2[0]-pt1[0]) + (pt2[1]-pt1[1])*(pt2[1]-pt1[1]))

	def inRo(self,val):
		return int(round(val))

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

	# def Heuristic2(self,pt):
	# 	HeuPt_ind = self.finHeuPt(pt)
	# 	return self.distance(pt,self.path_heu_pts[HeuPt_ind]) + self.path_heu_ptsUNdist[HeuPt_ind][1]

	def Heuristic(self,pt):
		return self.distance(pt,[self.inRo(pt[0]),self.inRo(pt[1])]) + self.costs_heuristic[self.inRo(pt[1])][self.inRo(pt[0])]

	# def Heuristic(self,pt):
	# 	return self.costs_heuristic[self.inRo(pt[1])][self.inRo(pt[0])]

	def create_actionSet(self,node,orientation):
		len_veh = self.len_veh
		velocity = self.velocity
		time_step = self.time_step

		loc = node	
		actSet = []
		alpha = -30

		while(alpha <= 30):
			if(round(alpha) != 0):
				radi = len_veh/math.tan(alpha*np.pi/180)
				orientation_new = orientation + (velocity*math.tan(alpha*np.pi/180)/len_veh)*(time_step)
				loc_new = [loc[0] + (len_veh/math.tan(alpha*np.pi/180))*(math.sin(orientation_new) - math.sin(orientation)), loc[1] + (len_veh/math.tan(alpha*np.pi/180))*(math.cos(orientation) - math.cos(orientation_new))]

				centre_botFram = [0,radi]
				theta_trans = -orientation
				loc_trans = [-loc[0],-loc[1]]
				centre_global = [radi*math.sin(theta_trans) - loc_trans[0], radi*math.cos(theta_trans) - loc_trans[1]]
				actSet.append([loc_new,1.03*time_step,orientation_new,alpha*np.pi/180,centre_global])

			else:
				loc_new = [loc[0] + velocity*math.cos(orientation)*time_step,loc[1] + velocity*math.sin(orientation)*time_step]
				orientation_new = orientation
				centre_global = [np.nan,np.nan]
				actSet.append([loc_new,time_step,orientation_new,alpha*np.pi/180,centre_global])

			alpha = alpha + 5

		return actSet       

	def count_ninetys(self,ang1,ang2):
		ninetys = [-np.pi,-np.pi/2,0,np.pi/2,np.pi,3*np.pi/2,2*np.pi,5*np.pi/2,3*np.pi,7*np.pi/2,4*np.pi]
		ninetys_in = []

		for ang in ninetys:
			if(ang1 < ang and ang < ang2):
				ninetys_in.append(ang)

		# print("Ninety Angles : ",np.dot(ninetys_in,180/np.pi).tolist())
		return ninetys_in

	def count_end_pts(self,loc1,loc2,alpha,centre):

		len_veh = 2/math.sqrt(3)
		velocity = 1
		time_step = 1
		pts_x_y = []

		if(round(alpha*180/np.pi) != 0):
			radi = len_veh/math.tan(alpha)
			time_rev = 2*np.pi*abs(radi)/velocity

			if(time_step < time_rev):
				pts_x_y = [[loc1[0],loc2[0]],[loc1[1],loc2[1]]]
				ang1 = math.atan2(loc1[1] - centre[1],loc1[0] - centre[0])
				ang2 = math.atan2(loc2[1] - centre[1],loc2[0] - centre[0])

				if(math.tan(alpha) > 0):
					if(ang2 < ang1):
						ang2 = ang2 + 2*np.pi

					angs_in = self.count_ninetys(ang1,ang2)
					for nine_ang in angs_in:
						pts_x_y[0].append(centre[0] + abs(radi)*math.cos(nine_ang))
						pts_x_y[1].append(centre[1] + abs(radi)*math.sin(nine_ang))

				else:
					if(ang1 < ang2):
						ang1 = ang1 + 2*np.pi

					angs_in = self.count_ninetys(ang2,ang1)
					for nine_ang in angs_in:
						pts_x_y[0].append(centre[0] + abs(radi)*math.cos(nine_ang))
						pts_x_y[1].append(centre[1] + abs(radi)*math.sin(nine_ang))

			else:
				pts_x_y = [[centre[0] - abs(radi),centre[0] + abs(radi),centre[0],centre[0]],[centre[1],centre[1],centre[1] - abs(radi),centre[1] + abs(radi)]]

		else:
			pts_x_y = [[loc1[0],loc2[0]],[loc1[1],loc2[1]]]

		return pts_x_y

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
				cv2.ellipse(img,(self.inRo(centre[0]),self.inRo(centre[1])),(self.inRo(abs(radi)),self.inRo(abs(radi))),0,ang1*180/np.pi,ang2*180/np.pi,155,1)

			else:
				cv2.circle(img,(self.inRo(centre[0]),self.inRo(centre[1])),self.inRo(abs(radi)),155,1)

		else:
			cv2.line(img,(self.inRo(loc1[0]),self.inRo(loc1[1])),(self.inRo(loc2[0]),self.inRo(loc2[1])),155,1)

	def obstacle_check(self,node1,node2,alpha,centre):
		
		end_pts = self.count_end_pts(node1,node2,alpha,centre)

		x_list = end_pts[0]
		y_list = end_pts[1]

		x_mi = self.inRo(np.amin(x_list)) - 3
		if(x_mi < 0):
			x_mi = 0

		x_ma = self.inRo(np.amax(x_list)) + 3
		if(x_ma >= self.img_w):
			x_ma = self.img_w - 1

		y_mi = self.inRo(np.amin(y_list)) - 3
		if(y_mi < 0):
			y_mi = 0

		y_ma = self.inRo(np.amax(y_list)) + 3
		if(y_ma >= self.img_h):
			y_ma = self.img_h - 1


		img_obst_roi = self.img[y_mi:y_ma,x_mi:x_ma]

		origin_roi = [x_mi,y_mi]

		node1_new = [node1[0] - origin_roi[0],node1[1] - origin_roi[1]]
		node2_new = [node2[0] - origin_roi[0],node2[1] - origin_roi[1]]
		centre_new = [centre[0] - origin_roi[0],centre[1] - origin_roi[1]]

		win_action = np.zeros(img_obst_roi.shape,np.uint8)
		self.disp_action(win_action,node1_new,node2_new,alpha,centre_new)

		obs_coll = cv2.bitwise_and(win_action,img_obst_roi)
		if(np.amax(obs_coll) > 10):
			return True

		elif(node2[0] < 0 or node2[0] >= self.img_w or node2[1] < 0 or node2[1] >= self.img_h):
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

		start_node = tree(start,self.orientation,0,[np.nan,np.nan],0,self.Heuristic(start)/self.velocity)

		que_open.push(start_node)                       #que_open is created for popping an element in each iteration 
		dict_open[start_node] = start_node              #dict_open is created for checking whether an element is in the OPEN list.
		time1 = time.time()
		time_prev = 0

		i = 0
		while(que_open.leng() > 0):
			node_current = que_open.pop()           #popping element

			del(dict_open[node_current])            #deleting it from the dictionary
			loc = node_current.location

			if(self.distance(node_current.location,goal) < 20):     #breaks the loop if it reaches goal
				solution = True
				break

			closed.append(node_current)                             #adding the popped element to closed 
			dict_closed[node_current] = node_current                #adding the popped element to closed dictionary
			action_set = self.create_actionSet(node_current.location,node_current.orientation)         #creating action set

			# cv2.imshow('image',img)
			# key = cv2.waitKey(1)
			# if(key == 27):
			# 	break

			img_action = self.img.copy()
			for action in action_set:                               #running the loop for creating new children


				img_act_disp = img.copy()

				node_successor_loc = action[0]           
 				node_successor_g = node_current.g + action[1]
				node_successor_f = node_successor_g + (self.Heuristic(node_successor_loc)/self.velocity)

				node_successor_orien = action[2]

				node_successor = tree(node_successor_loc,node_successor_orien,action[3],action[4],node_successor_g,node_successor_f)

				node_successor.parent = node_current
			
				val_open = node_successor in dict_open
				val_closed = node_successor in dict_closed
				valObs = self.obstacle_check(node_current.location,node_successor_loc,action[3],action[4])

				if(valObs == False):
					self.disp_action(img,node_current.location,action[0],action[3],action[4])
					cv2.circle(img,(self.inRo(node_current.location[0]),self.inRo(node_current.location[1])),2,155,-1)
					cv2.circle(img,(self.inRo(node_successor_loc[0]),self.inRo(node_successor_loc[1])),2,155,-1)

					self.disp_action(img_action,node_current.location,action[0],action[3],action[4])
					cv2.circle(img_action,(self.inRo(node_current.location[0]),self.inRo(node_current.location[1])),2,155,-1)
					cv2.circle(img_action,(self.inRo(node_successor_loc[0]),self.inRo(node_successor_loc[1])),2,155,-1)

					if(val_closed == False):
						if(val_open == False):
							que_open.push(node_successor)
							dict_open[node_successor] = node_successor

						else:
							open_successor = dict_open[node_successor]
							if(node_successor.g < open_successor.g):      
								open_successor.g = node_successor.g
								open_successor.f = node_successor.f
								open_successor.alpha = node_successor.alpha
								open_successor.centre = node_successor.centre
								open_successor.parent = node_successor.parent
								open_successor.orientation = node_successor.orientation


			time_curr = time.time() - time1
			if(round(time_curr*10)%10 == 0 and round(time_curr*10)/10 != round(time_prev*10)/10):
				print(str(int(time_curr/60)) + ' min, ' + str(int(time_curr%60)) + ' sec')
				time_prev = round(time_curr*10)/10

			i = i + 1

		time2 = time.time()

		print("Path Length : ",node_current.f)
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
# img = np.zeros((500,500),np.uint8)

inf_stgo = [[90,465],[470,460]]
obj = Astar_BiCycle(inf_stgo,-np.pi/2,img)
solution,img_main,path_pts = obj.main_class()
print('###############')

imgC = img.copy()
cv2.circle(imgC,(obj.inRo(path_pts[0][1][0]),obj.inRo(path_pts[0][1][1])),2,155,-1)
for pt in path_pts:
	# print('Par : ' + str(pt[0]) + ', Curr : ' + str(pt[1]) + ', Alpha : ' + str(pt[2]*180/np.pi))
	# print('-------------------------')
	obj.disp_action(imgC,pt[0],pt[1],pt[2],pt[3])
	cv2.circle(imgC,(obj.inRo(pt[0][0]),obj.inRo(pt[0][1])),2,155,-1)
	cv2.imshow('image',imgC)
	cv2.waitKey(100)

cv2.waitKey(0)

# Obstacle Check function needs to be modified, getting inf cost (for grassFire heuristic) for few nodes.
# Implement Voronoi Fields or Artificial Potential Fields.
# If time supports, then try the idea in replace of Grassfire.