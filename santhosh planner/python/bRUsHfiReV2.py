import cv2
import numpy as np 
from numpy import unravel_index
import math 
import time
import csv


kt45 = []
kt67 = []
kt89 = []
kt1011 = []
kt112 = []
kt012 = []

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
		t0 = time.time()
		global kt45,kt67,kt89,kt1011,kt112,kt012

		list_pts_bound = []
		list_vals_bound = []
		list_pts_empty = []

		neighs_perp = [[0,1],[0,-1],[1,0],[-1,0]]

		t1 = time.time()
		k = 0
		while(k < len(neighs_perp)):
			[i,j] = neighs_perp[k]

			t4 = time.time()		
			ind = [pt[0] + i,pt[1] + j]
			t5 = time.time()		
			kt45.append(t5-t4)
			if(ind[0] >= 0 and ind[1] >= 0 and ind[0] < potentials_map.shape[1] and ind[1] < potentials_map.shape[0]):
				t6 = time.time()		
				pot_val_ind = potentials_map[ind[1]][ind[0]]
				t7 = time.time()		
				kt67.append(t7-t6)
				if(pot_val_ind > 0):
					t8 = time.time()		
					list_pts_bound.append(ind)

					# list_vals_bound.append(potentials_map[ind[1]][ind[0]] + distance(pt,ind))
					list_vals_bound.append(potentials_map[ind[1]][ind[0]] + 1)
					t9 = time.time()		
					kt89.append(t9-t8)

				elif(self.inRo(pot_val_ind) == 0):
					t10 = time.time()		
					list_pts_empty.append(ind)
					t11 = time.time()		
					kt1011.append(t11-t10)

			k = k + 1

		t12 = time.time()
		kt112.append(t12-t1)
		kt012.append(t12-t0)

		return list_pts_empty,list_vals_bound,list_pts_bound

	def getboundObs(self,bound_pts):
		img = self.img.copy()

		bound_pts_obs = []

		neighs = [[-1,0],[0,1],[1,0],[0,-1],[-1,-1],[-1,1],[1,1],[1,-1]] #do not change or modify the arrangement in the list.

		for pt in bound_pts:
			i = 0
			while(i < len(neighs)):
				pt_neigh = [pt[0] + neighs[i][0],pt[1] + neighs[i][1]]
				if(pt_neigh[0] >= 0 and pt_neigh[0] < img.shape[1] and pt_neigh[1] >= 0 and pt_neigh[1] < img.shape[0]):
					if(img[pt_neigh[1]][pt_neigh[0]] == 255):
						bound_pts_obs.append(pt_neigh)
						break
				i = i + 1

		if(len(bound_pts_obs) != len(bound_pts)):
			print("Error in finding Obstacles of boundary points.")

		return bound_pts_obs

	def main(self):
		img = self.img.copy()
		img_disp = img.copy()

		t1 = time.time()
		bound_pts = self.getBoundPts(img)
		t2 = time.time()
		# print("1-2 : ",t2-t1)
		brush_que = bound_pts[:]
		brush_que_dict = {}


		mat_potential = img.copy().astype('float64')
		t3 = time.time()
		# print("2-3 : ",t3-t2)
		mat_potential = np.dot(mat_potential,-1/255)
		t4 = time.time()
		# print("3-4 : ",t4-t3)
		boundObsLoc = self.getboundObs(bound_pts)
		t5 = time.time()
		# print("4-5 : ",t5-t4)
		obsLoc_mat_pot = {}

		t6 = time.time()
		# print("5-6 : ",t6-t5)
		i = 0
		while(i < len(bound_pts)):
			pt = bound_pts[i]
			pt_obs = boundObsLoc[i]
			mat_potential[pt[1]][pt[0]] = 1
			obsLoc_mat_pot[hash(tuple(pt))] = pt_obs
			brush_que_dict[hash(tuple(pt))] = pt

			i = i + 1


		t78 = []
		t89 = []
		t910 = []
		t1011 = []

		itr = 0
		while(True):
			t7 = time.time()
			if(len(brush_que) == 0):
				break

			pt_ex = brush_que[0]
			del brush_que[0]
			del brush_que_dict[hash(tuple(pt_ex))]

			t8 = time.time()
			t78.append(t8-t7)
			neighs_empty,neighs_val,neighs_pts = self.getNeighPts(pt_ex,mat_potential)
			t9 = time.time()
			t89.append(t9-t8)
			if(mat_potential[pt_ex[1]][pt_ex[0]] == 0):
				near_neigh = neighs_pts[np.argmin(neighs_val)]

				mat_potential[pt_ex[1]][pt_ex[0]] = mat_potential[near_neigh[1]][near_neigh[0]] + 1
				obsLoc_mat_pot[hash(tuple(pt_ex))] = obsLoc_mat_pot[hash(tuple(near_neigh))]

			t10 = time.time()
			t910.append(t10-t9)

			for pt_emp in neighs_empty:
				if(hash(tuple(pt_emp)) in brush_que_dict):
					pass
				else:
					brush_que.append(pt_emp)
					brush_que_dict[hash(tuple(pt_emp))] = pt_emp

			t11 = time.time()
			t1011.append(t11-t10)
			img_disp[pt_ex[1]][pt_ex[0]] = 75 + 3*mat_potential[pt_ex[1]][pt_ex[0]]

			itr = itr + 1

		return mat_potential,obsLoc_mat_pot,img_disp



img = cv2.imread('MaInmAP.png',0)
# img = cv2.imread('MaInmAP_mINi2.png',0)
# img[0:325,:] = 255


# img = np.zeros((100,100),np.uint8)
# img[25:75,25:75] = 255
# img[3][3] = 255
# img[5][5] = 255
# img[7][7] = 255

# t1 = time.time()
# obj_obsDist = brushFire(img.copy())
# map_potential,obsLoc_mat_pot,img_potential = obj_obsDist.main()




# t2 = time.time()
# print(t2 - t1)

# map_potential,obsLoc_mat_pot,_ = obj_obsDist.main()
# # print("brushFire gENerAteD")

# cv2.imshow('image',img_potential)
# cv2.waitKey(0)

#########################################################################
# verify the distance from each point to obstacle with manhattan distance between the point and nearest obstacle.
# The algorithm works fine.
#########################################################################

