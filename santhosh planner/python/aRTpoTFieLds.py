import cv2
import numpy as np 
from numpy import unravel_index
import math 
import time

from gRaSSfiRe import *
from bRUsHfiRe import *


def minInd(pts):
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

def genPath(start,goal,mat_costs,img):
	curr = start[:]
	imgCp = img.copy()

	while(True):
		if(curr[0] == goal[0] and curr[1] == goal[1]):
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
		ind_lowNeigh = minInd(mat_window)
		ind_lowNeigh_t = unravel_index(mat_window.argmin(), mat_window.shape)

		# print("Diff : ",[ind_lowNeigh[0] - ind_lowNeigh_t[0],ind_lowNeigh[1] - ind_lowNeigh_t[1]])
		# print("Index : ",ind_lowNeigh)

		ind_lowNeigh = np.add(ind_lowNeigh,[y_low,x_low]).tolist()
		ind_lowNeigh.reverse()

		# print(ind_lowNeigh)
		imgCp[ind_lowNeigh[1]][ind_lowNeigh[0]] = 155
		curr = ind_lowNeigh

		cv2.imshow('image',imgCp)
		cv2.waitKey(1)

	return imgCp


img = cv2.imread('MaInmAP.png',0)
inf_stgo = [[90,465],[470,460]]

start = inf_stgo[0]
goal = inf_stgo[1] 

grassObj = grassFire(img,goal,1)
costs_grass = grassObj.genCosts()
print("Grassfire Heuristic Generated")

brushObj = brushFire(img)
costs_brush = brushObj.main()
brush_max = np.max(costs_brush)
print("Brushfire Heuristic Generated")

cost_map = np.inf*np.ones(img.shape)

i = 0
while(i < img.shape[1]):
	j = 0
	while(j < img.shape[0]):
		if(img[j][i] == 0):
			cost_map[j][i] = costs_grass[j][i] + 1*(brush_max/costs_brush[j][i])
		j = j + 1
	i = i + 1

img_path = genPath(start,goal,cost_map,img)
cv2.imshow('image',img_path)
cv2.waitKey(0)