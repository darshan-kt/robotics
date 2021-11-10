import cv2
import numpy as np 
from numpy import unravel_index
import math 
import time


def getBoundPts(img):
	kernel = np.ones((3,3), np.uint8) 
	img_dilate = cv2.dilate(img.copy(), kernel, iterations=1)
	img_bound = img_dilate - img.copy()
	_,bound_pts_cv, hierarchy = cv2.findContours(img_bound, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
	bound_pts = np.vstack(bound_pts_cv).squeeze()

	return bound_pts.tolist()

def inRo(val):
	return int(round(val))

def distance(pt1,pt2):
	return np.sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2)

img_color = cv2.imread('MaInmAP.png')
img = cv2.imread('MaInmAP.png',0)

bound_pts = getBoundPts(img)

t2 = time.time()
print(t2-t1)
print(bound_pts)



