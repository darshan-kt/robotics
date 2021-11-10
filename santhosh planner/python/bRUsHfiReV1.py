import cv2
import numpy as np 
from numpy import unravel_index
import math 
import time
import csv

class brushFire:
	def __init__(self,img):
		self.img = img

	def getBoundPts(self,img):
		kernel = np.ones((3,3), np.uint8) 
		img_dilate = cv2.dilate(img.copy(), kernel, iterations=1)
		img_bound = img_dilate - img.copy()

		bound_pts_cv = np.where(img_bound == 255)

		bound_pts = []
		i = 0
		while(i < len(bound_pts_cv[0])):
			bound_pts.append([bound_pts_cv[1][i],bound_pts_cv[0][i]])
			i = i + 1

		return bound_pts

	def inRo(self,val):
		return int(round(val))

	def distance(self,pt1,pt2):
		return np.sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2)

	def getNeighPts(self,pt,potentials_map):
		list_pts_bound = []
		list_vals_bound = []
		list_pts_empty = []

		i = -1
		while(i < 2):
			j = -1
			while(j < 2):
				if((i == 0 and j == 0) or (abs(i) == 1 and abs(j) == 1)):
					pass
				else:
					ind = [pt[0] + i,pt[1] + j]
					if(ind[0] >= 0 and ind[1] >= 0 and ind[0] < potentials_map.shape[1] and ind[1] < potentials_map.shape[0]):
						pot_val_ind = potentials_map[ind[1]][ind[0]]
						if(pot_val_ind > 0):
							list_pts_bound.append(ind)

							# list_vals_bound.append(potentials_map[ind[1]][ind[0]] + distance(pt,ind))
							list_vals_bound.append(potentials_map[ind[1]][ind[0]] + 1)

						elif(self.inRo(pot_val_ind) == 0):
							list_pts_empty.append(ind)

				j = j + 1
			i = i + 1

		return list_pts_empty,list_vals_bound,list_pts_bound

	def main(self):
		img = self.img.copy()
		img_disp = img.copy()

		bound_pts = self.getBoundPts(img)
		brush_que = bound_pts[:]
		brush_que_dict = {}

		mat_potential = img.copy().astype('float64')
		mat_potential = np.dot(mat_potential,-10/255)

		for pt in bound_pts:
			mat_potential[pt[1]][pt[0]] = 1
			brush_que_dict[hash(tuple(pt))] = pt


		# Do not delete this
		# Do not change anything from here------------------------------

		# neighs_bound_pt = [[0,1],[1,0],[0,-1],[-1,0],[-1,-1],[-1,1],[1,-1],[1,1]] 
		# for pt in bound_pts:
		# 	for pt_neigh in neighs_bound_pt:
		# 		if(pt[0]+pt_neigh[0] >= 0 and pt[0]+pt_neigh[0] < mat_potential.shape[1]  and pt[1]+pt_neigh[1] >= 0 and pt[1]+pt_neigh[1] < mat_potential.shape[0]):
		# 			if(img[pt[1]+pt_neigh[1]][pt[0]+pt_neigh[0]] == 255):
		# 				if(pt_neigh[0] == 0 or pt_neigh[1] == 0):
		# 					mat_potential[pt[1]][pt[0]] = 1
		# 					break
		# 				else:
		# 					mat_potential[pt[1]][pt[0]] = math.sqrt(2)

		# ---------------------------------------------------------------till here



		while(True):
			if(len(brush_que) == 0):
				# print("mAp gENerAteD")
				break

			pt_ex = brush_que[0]
			del brush_que[0]
			del brush_que_dict[hash(tuple(pt_ex))]

			# cv2.imshow('image',img_disp)
			# key = cv2.waitKey(1)
			# if(key == ord(' ')):
			# 	cv2.waitKey(0)
			# elif(key == 27):
			# 	break

			neighs_empty,neighs_val,neighs_pts = self.getNeighPts(pt_ex,mat_potential)
			if(mat_potential[pt_ex[1]][pt_ex[0]] == 0):
				near_neigh = neighs_pts[np.argmin(neighs_val)]

				# mat_potential[pt_ex[1]][pt_ex[0]] = mat_potential[near_neigh[1]][near_neigh[0]] + distance(near_neigh,pt_ex)
				mat_potential[pt_ex[1]][pt_ex[0]] = mat_potential[near_neigh[1]][near_neigh[0]] + 1
		 
			for pt_emp in neighs_empty:
				if(hash(tuple(pt_emp)) in brush_que_dict):
					pass
				else:
					brush_que.append(pt_emp)
					brush_que_dict[hash(tuple(pt_emp))] = pt_emp

			img_disp[pt_ex[1]][pt_ex[0]] = 75 + 3*mat_potential[pt_ex[1]][pt_ex[0]]

		return mat_potential
		# return mat_potential,img_disp


# img = cv2.imread('MaInmAP.png',0)
# # img = cv2.imread('MaInmAP_mINi2.png',0)
# # img[0:325,:] = 255

# obj_obsDist = brushFire(img.copy())
# map_potential, img_potential = obj_obsDist.main()

# cv2.imshow('image',img_potential)
# cv2.waitKey(0)
