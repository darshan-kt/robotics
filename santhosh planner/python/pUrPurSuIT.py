import cv2
import numpy as np 
import math
import time
import random
from scipy import spatial


class pure_pursuit:
	def __init__(self,bot_loc,bot_orien,goal,path_pts,velocity,len_veh,lookAhead):
		self.bot_loc = bot_loc
		self.bot_orien = bot_orien
		self.path_pts = path_pts
		self.velocity = velocity
		self.len_veh = len_veh
		self.lookAhead = lookAhead
		self.time_step = 0.1
		self.goal = goal
		self.steer_max = np.pi/6

	def distance(self,pt1,pt2):
		return math.sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2)

	def evalEta(self,bot_loc,bot_orien,loc_lookAhead):

		bot_loc = np.transpose(np.matrix(bot_loc))
		loc_lookAhead = np.transpose(np.matrix(loc_lookAhead))
		trans_mat = np.matrix([[math.cos(bot_orien),math.sin(bot_orien)],[-math.sin(bot_orien),math.cos(bot_orien)]])

		loc_lookAhead_translation = loc_lookAhead - bot_loc
		loc_lookAhead_transform = np.matmul(trans_mat,loc_lookAhead_translation)


		eta = -math.atan2(loc_lookAhead_transform[1],loc_lookAhead_transform[0])

		return eta

	def evalAlpha(self,bot_loc,bot_orien,loc_lookAhead):
		Lfw = self.distance(bot_loc,loc_lookAhead)

		eta = self.evalEta(bot_loc,bot_orien,loc_lookAhead)
		alpha = -math.atan(2*self.len_veh*math.sin(eta)/Lfw)

		return alpha

	def getNearPt(self,bot_loc):
		path_pts = np.array(self.path_pts)
		dists = ((path_pts-np.array(bot_loc))**2).sum(axis=1)

		ind = np.argmin(dists)

		return ind,path_pts[ind].tolist()

	def getLookaheadInd(self,ind_nearPt):

		path_pts = self.path_pts[:]
		ind_lookAhead = ind_nearPt
		while(ind_lookAhead < len(path_pts)):
			if(self.distance(path_pts[ind_nearPt],path_pts[ind_lookAhead]) >= self.lookAhead):
				return ind_lookAhead,path_pts[ind_lookAhead]

			ind_lookAhead = ind_lookAhead + 1

		return ind_lookAhead-1,path_pts[ind_lookAhead-1]

	def predPt(self,bot_loc,bot_orien,alpha):
		alpha = alpha*180/np.pi

		if(round(alpha) != 0):
			bot_orien_new = bot_orien + (self.velocity*math.tan(alpha*np.pi/180)/self.len_veh)*(self.time_step)
			bot_loc_new = [bot_loc[0] + (self.len_veh/math.tan(alpha*np.pi/180))*(math.sin(bot_orien_new) - math.sin(bot_orien)), bot_loc[1] + (self.len_veh/math.tan(alpha*np.pi/180))*(math.cos(bot_orien) - math.cos(bot_orien_new))]

		else:
			bot_orien_new = bot_orien
			bot_loc_new = [bot_loc[0] + self.velocity*math.cos(bot_orien)*self.time_step,bot_loc[1] + self.velocity*math.sin(bot_orien)*self.time_step]

		return bot_loc_new,bot_orien_new

	def genPursuit(self):
		bot_loc = self.bot_loc[:]
		bot_orien = self.bot_orien
		path_pts = self.path_pts[:]

		path_pursuit = [bot_loc]

		i = 0
		while(True):
			if(self.distance(bot_loc,self.goal) < 10):
				break

			ind_nearPt,nearPt = self.getNearPt(bot_loc)

			ind_lookAhead,pt_lookAhead = self.getLookaheadInd(ind_nearPt)

			eta = self.evalAlpha(bot_loc,bot_orien,pt_lookAhead)

			alpha = self.evalAlpha(bot_loc,bot_orien,pt_lookAhead)


			if(abs(alpha) > self.steer_max):
				alpha = np.pi/6*abs(alpha)/alpha

			bot_loc_new,bot_orien_new = self.predPt(bot_loc,bot_orien,alpha)
			path_pursuit.append(bot_loc)

			bot_loc = bot_loc_new[:]
			bot_orien = bot_orien_new

			i = i + 1
		return path_pursuit

# def inRo(val):
# 	return int(round(val))

# path_pts = []

# i = 100	
# while(i <= 400):
# 	path_pts.append([i,100])
# 	i = i + 1

# i = 400
# j = 100
# while(i >= 100 and j <= 400):
# 	path_pts.append([i,j])

# 	j = j + 1
# 	i = i - 1



# t1 = time.time()
# pure_pursObj = pure_pursuit(path_pts[0],0,path_pts[-1],path_pts,40,5,10)
# path_pursuit = pure_pursObj.genPursuit()
# t2 = time.time()


# print(t2-t1)

# img = np.zeros((500,500),np.uint8)

# for pt in path_pts:
# 	img[pt[1]][pt[0]] = 255

# for pt in path_pursuit:
# 	img[inRo(pt[1])][inRo(pt[0])] = 155

# cv2.imshow('image',img)
# cv2.waitKey(0)

