import cv2
import numpy as np 
import math
import time
import ast 
import csv
from Queue import PriorityQueue


class tree:
	def __init__(self,location,g,f):
		self.location = location
		self.parent = None
		self.g = g
		self.f = f

	def __hash__(self):
		return hash((math.floor(self.location[0]),math.floor(self.location[1])))

	def __eq__(self,other):
		return (math.floor(self.location[0]),math.floor(self.location[1])) == (math.floor(other.location[0]),math.floor(other.location[1]))

	def __le__(self,other):
		self_val = self.f
		other_val = other.f
		return self_val <= other_val

	def __lt__(self,other):
		self_val = self.f
		other_val = other.f
		return self_val < other_val


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


class Astar:
	def __init__(self,stGo,img):
		self.stGo = stGo                           
		self.img = img
		self.img_h,self.img_w = img.shape

	def distance(self,pt1,pt2):
		return np.sqrt((pt2[0]-pt1[0])*(pt2[0]-pt1[0]) + (pt2[1]-pt1[1])*(pt2[1]-pt1[1]))

	def pt_shift(self,node,list_nodes):
		list_ret = []
		i = 0
		while (i < len(list_nodes)):
			list_ret.append([list_nodes[i][0]-node[0],list_nodes[i][1]-node[1]])
			i = i + 1

		return list_ret	

	def create_action(self,node):

		action_set = []
		angle = 0
	
		ex = -np.pi
		while(ex < np.pi):		
			pt = [20*math.cos(angle + ex),20*math.sin(angle + ex)]
			pt_shift = self.pt_shift([-node[0],-node[1]],[pt])
			
			action = [pt_shift,20]

			action_set.append(action)
			ex = ex + np.pi/18

		return action_set

	def obstacle_check(self,node1,node2):
		x_list = [node1[0],node2[0]]
		y_list = [node1[1],node2[1]]

		x_mi = int(np.amin(x_list)) - 3
		if(x_mi < 0):
			x_mi = 0

		x_ma = int(np.amax(x_list)) + 3
		if(x_ma >= self.img_w):
			x_ma = self.img_w - 1

		y_mi = int(np.amin(y_list)) - 3
		if(y_mi < 0):
			y_mi = 0
		y_ma = int(np.amax(y_list)) + 3
		if(y_ma >= self.img_h):
			y_ma = self.img_h - 1


		img_obs_roi = self.img[y_mi:y_ma,x_mi:x_ma]

		orig_roi = [x_mi,y_mi]

		node1_new = [node1[0] - orig_roi[0],node1[1] - orig_roi[1]]
		node2_new = [node2[0] - orig_roi[0],node2[1] - orig_roi[1]]

		win_path = np.zeros(img_obs_roi.shape,np.uint8)
		cv2.line(win_path,(int(node1_new[0]),int(node1_new[1])),(int(node2_new[0]),int(node2_new[1])),255,2)

		obs_coll = cv2.bitwise_and(win_path,img_obs_roi)
		if(np.amax(obs_coll) > 10):
			return True

		elif(node2[0] < 0 or node2[0] >= self.img_w or node2[1] < 0 or node2[1] >= self.img_h):
			return True

		else:
			return False

	def main_class(self):                                   

		time1 = time.time()
		time_prev = 0

		img = self.img.copy()
		inf_stgo = self.stGo
		cv2.circle(img,tuple(inf_stgo[0]),3,255,-1)
		cv2.circle(img,tuple(inf_stgo[1]),3,255,-1)
		cv2.circle(img,tuple(inf_stgo[1]),20,255,1)
		start = inf_stgo[0]
		goal = inf_stgo[1]

		que_open = Queue()
		closed = []
		dict_open = {}
		dict_closed = {}        

		solution = False
		start_node = tree(start,0,self.distance(start,goal))
		que_open.push(start_node)                        
		dict_open[start_node] = start_node              

		while(que_open.leng() > 0):

			node_current = que_open.pop()           
			del(dict_open[node_current])            
			loc = node_current.location

			if(self.distance(node_current.location,goal) < 20):
				solution = True
				break

			closed.append(node_current)
			dict_closed[node_current] = node_current
			action_set = self.create_action(node_current.location)

			for action in action_set:

				node_successor_loc = action[0][0]           
				node_successor_g = node_current.g + action[1]
				node_successor_f = node_successor_g + self.distance(node_successor_loc,goal)

				node_successor = tree(node_successor_loc,node_successor_g,node_successor_f)
				node_successor.parent = node_current
			
				val_open = node_successor in dict_open
				val_closed = node_successor in dict_closed

				t3 = time.time()
				val0 = self.obstacle_check(node_current.location,node_successor_loc)
				t4 = time.time()

				if(val0 == False):
					if(val_closed == False):
						if(val_open == False):
							que_open.push(node_successor)
							dict_open[node_successor] = node_successor
							cv2.circle(img,tuple(np.asarray(node_successor.location,np.int)),2,255,-1)
							cv2.line(img,tuple(np.asarray(node_successor.location,np.int)),tuple(np.asarray(node_current.location,np.int)),255,1)
						else:
							open_successor = dict_open[node_successor]
							if(node_successor.g < open_successor.g):
								open_successor.g = node_successor.g
								open_successor.parent = node_successor.parent
								open_successor.f = node_successor.f

				# cv2.imshow('image_A*',img)
				# cv2.waitKey(1)

			time_curr = time.time() - time1
			if(round(time_curr*10)%10 == 0 and round(time_curr*10)/10 != round(time_prev*10)/10):
				print(str(int(time_curr/60)) + ' min, ' + str(int(time_curr%60)) + ' sec')
				time_prev = round(time_curr*10)/10

		prev = goal
		pts_path = [goal]
		img2 = self.img.copy()
		while(solution == True):
			cur = node_current.location
			pts_path.append(cur)
			cv2.circle(img2,(int(cur[0]),int(cur[1])),2,255,-1)
			cv2.line(img2,(int(cur[0]),int(cur[1])),(int(prev[0]),int(prev[1])),255,1)
			if(node_current.location[0] == start[0] and node_current.location[1] == start[1]):
				cv2.circle(img2,(int(start[0]),int(start[1])),5,255,-1)
				cv2.circle(img2,(int(goal[0]),int(goal[1])),5,255,-1)
				break
			node_current = node_current.parent

			prev = cur

		pts_path.reverse()
		return solution,img2,pts_path


# # img = np.zeros((500,500),np.uint8)
# # img = cv2.imread('tEsTmAP.png',0)
# img = cv2.imread('MaInmAP_mINi.png',0)
# img = cv2.imread('MaInmAP.png',0)

# inf_stgo = [[90,465],[470,460]]
# obj = Astar(inf_stgo,img)
# solution,img_ret,path_pts = obj.main_class()

# # print("####################################")
# # print(path_pts)
# # print("####################################")

# while(True):
# 	cv2.imshow('image_A*',img_ret)
# 	if(cv2.waitKey(1)&0xFF == 27):
# 		break
