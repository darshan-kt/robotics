import cv2
import numpy as np 
from numpy import unravel_index
import math
import time
import csv


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

					# if(self.obstacle_check(pt,neighPt) == False):
					# 	neighGlob.append([neighPt,self.distance(pt,neighPt)])

				j = j + 1
			i = i + 1

		return neighGlob

	def minInd(self,pts):
		minVal = np.inf
		ind = [None,None]
		i = 0
		while(i < pts.shape[0]):
			j = 0
			while(j < pts.shape[1]):
				if(pts[i][j] < minVal):
					minVal = pts[i][j]
					ind = [i,j]
				j = j + 1
			i = i + 1

		return ind

	def genPath(self,start,mat_costs):
		curr = start[:]
		imgCp = self.img.copy()

		while(True):
			if(mat_costs[curr[1]][curr[0]] == 0):
				print('Path Generated')
				break

			x_low = curr[0]-1
			if(x_low < 0):
				x_low = 0

			x_high = curr[0]+2
			if(x_high > imgCp.shape[1]):
				x_high = imgCp.shape[1]

			y_low = curr[1]-1
			if(y_low < 0):
				y_low = 0

			y_high = curr[1]+2
			if(y_high > imgCp.shape[0]):
				y_high = imgCp.shape[0]

			mat_window = mat_costs[y_low:y_high,x_low:x_high]
			ind_lowNeigh = self.minInd(mat_window)
			ind_lowNeigh_t = unravel_index(mat_window.argmin(), mat_window.shape)

			# print("Diff : ",[ind_lowNeigh[0] - ind_lowNeigh_t[0],ind_lowNeigh[1] - ind_lowNeigh_t[1]])
			# print("Index : ",ind_lowNeigh)

			ind_lowNeigh = np.add(ind_lowNeigh,[y_low,x_low]).tolist()
			ind_lowNeigh.reverse()

			# print(ind_lowNeigh)
			imgCp[ind_lowNeigh[1]][ind_lowNeigh[0]] = 155
			curr = ind_lowNeigh

		return imgCp

	def genCosts(self):
		img = self.img.copy()
		img_disp = img.copy()
		loc_goal = self.loc_goal[:]

		mat_costs = np.dot(np.ones(img.shape),np.inf)		
		mat_costs[loc_goal[1]][loc_goal[0]] = 0
		list_open = [loc_goal]

		# i = 0
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





# img = np.zeros((500,500),np.uint8)
# img = cv2.imread('MaInmAP.png',0)
# # img = cv2.imread('MaInmAP_mINi.png',0)
# # img = cv2.imread('MaInmAP_mINi2.png',0)
# # img = cv2.imread('tEsTmAP.png',0)

# neighHoodSize = 1

# img = np.zeros((5,5),np.uint8)
# img[3][3] = 255
# grassObj = grassFire(img,[4,2],neighHoodSize)
# grassObj = grassFire(img,[470,460],neighHoodSize)
# mat_costs = grassObj.genCosts()
# print(mat_costs)
# print("(143,282) : ",mat_costs[282][143])
# print("(145,283) : ",mat_costs[283][145])
# print("(147,284) : ",mat_costs[284][147])

# img_path_grass = grassObj.genPath([90,465],mat_costs)
# print(t2-t1,t3-t2)
# cv2.imshow('image_grass',img_path_grass)
# cv2.waitKey(0)
# while(True):
# 	key = cv2.waitKey(1)
# 	if(key == 27):
# 		cv2.destroyAllWindows()
# 		break

# # file_name = "cOsTmAP.csv"
# # with open(file_name, 'w') as csvFile:
# #     writer = csv.writer(csvFile)
# #     writer.writerows(mat_costs[202:311,141:188])
# # csvFile.close()
# # print("AlL VaLuES sAVeD")

