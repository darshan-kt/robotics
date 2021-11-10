import cv2
import numpy as np 
import math 


class Dubins():
	def __init__(self,pt_node,pt_sample,pt_orientation,radius_min):
		self.pt_node = pt_node
		self.pt_sample = pt_sample
		self.pt_orientation = pt_orientation
		self.radius_min = radius_min

	def distance(self,pt1,pt2):
		return math.sqrt((pt1[0]-pt2[0])**2+(pt1[1]-pt2[1])**2)

	def point_newAxis(self,pt_node,pt_sample,orientation):
		theta = orientation*np.pi/180
		pt_sample = [pt_sample[0] - pt_node[0], pt_sample[1] - pt_node[1]]

		mat_trans = np.matrix([[math.cos(theta), math.sin(theta)],[-math.sin(theta), math.cos(theta)]])
		pt_sample_new = np.dot(mat_trans,np.transpose(pt_sample))
		pt_sample_new = pt_sample_new.tolist()
		
		return pt_sample_new[0]

	def distance_dubins(self):
		pt_node = self.pt_node
		pt_sample = self.pt_sample
		pt_orientation = self.pt_orientation
		sample = self.point_newAxis(pt_node,pt_sample,pt_orientation)
		r = self.radius_min
		x,y = sample[0], abs(sample[1])

		dist_from_center = self.distance([x,y],[0,r])
		length_dubins = 0
		if(dist_from_center >= r):
			length_dubins = self.dubin_out_distance(x,y,r)

		else:
			df_sample = self.distance([x,y],[0,-r])
			alpha_sample = 2*np.pi - math.acos((5*(r**2) - (df_sample**2))/(4*(r**2)))
			dist_paper = r*(alpha_sample + math.asin(x/df_sample) - math.asin((r*math.sin(alpha_sample))/df_sample))
			length_dubins = dist_paper

		return (length_dubins)

	def dubin_out_distance(self, x, y, r):
		alpha = np.arctan2(x,(r-y))
		altheta = np.arccos(r/np.sqrt(x*x + (y-r)**2))
		theta = round(alpha-altheta, 12) %(np.pi*2) # %(np.pi*2)

		return r*theta + np.sqrt((x-r*np.sin(theta))**2 + (y-r+r*np.cos(theta))**2)


