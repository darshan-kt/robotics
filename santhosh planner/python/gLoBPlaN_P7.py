import cv2
import numpy as np 
import math
import time
import csv

from tRee import *
from gRaSSfiRe import *
from bRUsHfiReV2 import *
from qUEue import *


class Astar_BiCycle:
	def __init__(self,stGo,orientation,img):
		self.stGo = stGo
		self.orientation = orientation     #list of start and goal locations
		self.img = img                                  #map for visualizing
		self.img_h,self.img_w = img.shape

		self.len_veh = 5
		self.velocity = 5
		self.time_step = 3

		t1 = time.time()
		grassObj = grassFire(img,stGo[1],1)
		self.costs_heuristic_goal = grassObj.genCosts()
		t2 = time.time()
		# print(t2-t1)
		print("gRaSSfiRe gEneRAtEd")
		print("#############################")

	def distance(self,pt1,pt2):                             #Euclidean Distance
		return np.sqrt((pt2[0]-pt1[0])*(pt2[0]-pt1[0]) + (pt2[1]-pt1[1])*(pt2[1]-pt1[1]))

	def inRo(self,val):
		return int(round(val))

	def Heuristic(self,pt):
		return self.distance(pt,[self.inRo(pt[0]),self.inRo(pt[1])]) + self.costs_heuristic_goal[self.inRo(pt[1])][self.inRo(pt[0])]

	# def Heuristic(self,pt):
	# 	return self.distance(pt,self.stGo[1])

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
		len_veh = self.len_veh
		velocity = self.velocity
		time_step = self.time_step
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
				pts_x_y = [[centre[0] - abs(radi),centre[0] + abs(radi)],[centre[1] - abs(radi),centre[1] + abs(radi)]]

		else:
			pts_x_y = [[loc1[0],loc2[0]],[loc1[1],loc2[1]]]

		return pts_x_y

	def disp_path(self,img_disp,node1,theta1,alpha):
		len_veh = self.len_veh
		velocity = self.velocity
		time_step = self.time_step
		img_map = self.img.copy()
		path_pts = []

		n = 30
		time_jump = float(time_step)/n

		i = 0
		while(i < n):
			theta2 = theta1 + (velocity*math.tan(alpha)/len_veh)*(time_jump)
			if(alpha != 0):
				node2 = [node1[0] + (len_veh/math.tan(alpha))*(math.sin(theta2) - math.sin(theta1)), node1[1] + (len_veh/math.tan(alpha))*(math.cos(theta1) - math.cos(theta2))]
			else:
				node2 = [node1[0] + velocity*math.cos(theta1)*time_jump,node1[1] + velocity*math.sin(theta1)*time_jump]

			if(self.inRo(node2[0]) >= 0 and self.inRo(node2[0]) < img_map.shape[1] and self.inRo(node2[1]) >= 0 and self.inRo(node2[1]) < img_map.shape[0]):
				path_pts.append(node2)
				img_disp[self.inRo(node2[1])][self.inRo(node2[0])] = 125

			node1 = node2
			theta1 = theta2
			i = i + 1

		return img_disp,path_pts

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
				cv2.ellipse(img,(self.inRo(centre[0]),self.inRo(centre[1])),(self.inRo(abs(radi)),self.inRo(abs(radi))),0,ang1*180/np.pi,ang2*180/np.pi,175,1)

			else:
				cv2.circle(img,(self.inRo(centre[0]),self.inRo(centre[1])),self.inRo(abs(radi)),175,1)

		else:
			cv2.line(img,(self.inRo(loc1[0]),self.inRo(loc1[1])),(self.inRo(loc2[0]),self.inRo(loc2[1])),175,1)

	def obstacle_check2(self,node1,theta1,alpha):
		len_veh = self.len_veh
		velocity = self.velocity
		time_step = self.time_step
		img_map = self.img.copy()

		n = 30
		time_jump = float(time_step)/n

		i = 0
		while(i <= n):
			theta2 = theta1 + (velocity*math.tan(alpha)/len_veh)*(time_jump)
			if(alpha != 0):
				node2 = [node1[0] + (len_veh/math.tan(alpha))*(math.sin(theta2) - math.sin(theta1)), node1[1] + (len_veh/math.tan(alpha))*(math.cos(theta1) - math.cos(theta2))]
			else:
				node2 = [node1[0] + velocity*math.cos(theta1)*time_jump,node1[1] + velocity*math.sin(theta1)*time_jump]

			if(self.inRo(node2[0]) >= 0 and self.inRo(node2[0]) < img_map.shape[1] and self.inRo(node2[1]) >= 0 and self.inRo(node2[1]) < img_map.shape[0]):
				if(img_map[self.inRo(node2[1])][self.inRo(node2[0])] == 255):
					return True
			else:
				return True

			node1 = node2
			theta1 = theta2
			i = i + 1

		return False

	def obstacle_check(self,node1,node2,alpha,centre,val_check):
		end_pts = self.count_end_pts(node1,node2,alpha,centre)

		x_list = end_pts[0]
		y_list = end_pts[1]

		x_mi = self.inRo(np.amin(x_list)) - 3
		if(x_mi < 0):
			x_mi = 0

		x_ma = self.inRo(np.amax(x_list)) + 4
		if(x_ma >= self.img_w):
			x_ma = self.img_w - 1

		y_mi = self.inRo(np.amin(y_list)) - 3
		if(y_mi < 0):
			y_mi = 0

		y_ma = self.inRo(np.amax(y_list)) + 4
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
		if(np.amax(obs_coll) > 0):
			return True

		elif(node2[0] < 0 or node2[0] >= self.img_w or node2[1] < 0 or node2[1] >= self.img_h or self.img[self.inRo(node2[1])][self.inRo(node2[0])] == 255):
			return True

		else:
			return False

	def getPathPts(self,solution,end_node):
		start = self.stGo[0]
		prev = end_node
		dts_path = []

		while(solution == True):
			cur = prev.parent
			dts_path.append([cur.location,cur.orientation,prev.location,prev.alpha,prev.centre])

			if(round(cur.location[0] - start[0]) == 0 and round(cur.location[1] - start[1]) == 0):
				break
			prev = cur

		dts_path.reverse()

		glob_path_pts = [start]
		for pt in dts_path:
			_,action_pts = self.disp_path(self.img.copy(),pt[0],pt[1],pt[3])
			glob_path_pts = glob_path_pts + action_pts

		return dts_path, glob_path_pts

	def main_class(self):                                   #main
		t1 = time.time()
		img = self.img.copy()
		inf_stgo = self.stGo
		cv2.circle(img,tuple(inf_stgo[0]),3,255,-1)
		cv2.circle(img,tuple(inf_stgo[1]),3,255,-1)
		cv2.circle(img,tuple(inf_stgo[1]),20,255,1)
		start = inf_stgo[0]                             #start location
		goal = inf_stgo[1]                              #goal location
		que_open = queue()                              #OPEN (priority queue)
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
		# while(i < 10):
		while(que_open.leng() > 0):
			# print("------------------------------")
			node_current = que_open.pop()           #popping element
			del(dict_open[node_current])            #deleting it from the dictionary
			loc = node_current.location

			if(self.distance(node_current.location,goal) < 20):     #breaks the loop if it reaches goal
				solution = True
				break

			closed.append(node_current)                             #adding the popped element to closed 
			dict_closed[node_current] = node_current                #adding the popped element to closed dictionary
			action_set = self.create_actionSet(node_current.location,node_current.orientation)         #creating action set

			img_action = self.img.copy()
			for action in action_set:                               #running the loop for creating new children

				img_act_disp = img.copy()

				node_successor_loc = action[0]           
				# print(node_successor_loc)
				node_successor_g = node_current.g + action[1]
				node_successor_f = node_successor_g + (self.Heuristic(node_successor_loc)/self.velocity)

				node_successor_orien = action[2]

				node_successor = tree(node_successor_loc,node_successor_orien,action[3],action[4],node_successor_g,node_successor_f)

				node_successor.parent = node_current
			
				val_open = node_successor in dict_open
				val_closed = node_successor in dict_closed

				# valObs = self.obstacle_check(node_current.location,node_successor_loc,action[3],action[4],self.Heuristic(node_successor_loc)/self.velocity)
				valObs = self.obstacle_check2(node_current.location,node_current.orientation,action[3])

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
				# print(str(int(time_curr/60)) + ' min, ' + str(int(time_curr%60)) + ' sec')
				time_prev = round(time_curr*10)/10

			i = i + 1

		t2 = time.time()
		# print(node_current.g)
		# print(t2-t1)
		way_pts, glob_path_pts = self.getPathPts(solution,node_current)
		return solution, way_pts, glob_path_pts


# ------------------------------------------------------------------------- #
# ##############################--------------############################# #
# ############################## End of Class ############################# #
# ##############################--------------############################# #
# ------------------------------------------------------------------------- #

	
# img = cv2.imread('MaInmAP.png',0)
# # img = cv2.imread('MaInmAP_mINi.png',0)
# img = cv2.imread('MaInmAP_mINi2.png',0)

# kernel = np.ones((3,3), np.uint8) 
# img_dilate = cv2.dilate(img.copy(), kernel, iterations=1)

# inf_stgo = [[90,465],[470,460]]      
# init_orien = -np.pi/2

# obj = Astar_BiCycle(inf_stgo,init_orien,img.copy())
# obj = Astar_BiCycle(inf_stgo,init_orien,img_dilate.copy())

# solution,way_path_pts,glob_path_pts = obj.main_class()

# imgC = img.copy()
# cv2.circle(imgC,tuple(inf_stgo[0]),3,255,-1)
# cv2.circle(imgC,tuple(inf_stgo[1]),3,255,-1)

# for pt in glob_path_pts:
# 	imgC[obj.inRo(pt[1])][obj.inRo(pt[0])] = 155
# 	cv2.imshow('image',imgC)
# 	cv2.waitKey(1)

# cv2.waitKey(0)

# Obstacle Check function needs to be modified, getting inf cost (for grassFire heuristic) for few nodes.
# -----------> The newly developed Obstacle Collision function (see in the backup functions) (obstacle_check2) works good but increases the Run time.
# -----------> UPDATE, bugs in the Old obstacle Collision fuction are fixed, use this for better run time. 
# Implement Voronoi Fields.
# If time supports, then try the idea in replace of Grassfire.