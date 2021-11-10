import cv2
import numpy as np 
import time
from gLoBPlaN_P7 import *
from bRUsHfiReV2 import *
from pUrPurSuIT import *


w_apf = 0
w_obsColl = 0.2
w_curv = 0
w_smth = 0.4

dmax = 20
leng_veh = 5
kmax = math.tan(np.pi/6)/leng_veh

def distance(pt1,pt2):
	return np.sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2)

def partDer_apf(xi,oi,d_xi):
	global w_apf,d_max

	alp = 100
	deriv = []
	k = 1										#tuning parameter

	if(distance(xi,oi) <= dmax):
		deriv_1 = -k*alp/((alp + d_xi)**2)	
		deriv_2 = np.dot([xi[0]-oi[0],xi[1]-oi[1]],1/distance(xi,oi)).tolist()
		deriv = np.dot(deriv_1,deriv_2).tolist()
	else:
		deriv = [0,0]

	return np.dot(w_apf,deriv).tolist()

def partDer_obsColl(xi,oi):
	global w_obsColl,dmax

	deriv = []

	if(distance(xi,oi) <= dmax):
		deriv_1 = 2*(distance(xi,oi) - dmax)
		deriv_2 = np.dot([xi[0]-oi[0],xi[1]-oi[1]],1/distance(xi,oi)).tolist()
		deriv = np.dot(deriv_1,deriv_2).tolist()
	else:
		deriv = [0,0]

	return np.dot(w_obsColl,deriv).tolist() 

def partDer_smth(i,path_pts):
	global w_smth

	deriv = []

	if(i == 0 or i == len(path_pts)-1):
		x_der = 0
		y_der = 0

	elif(i == 1):
		x_der = 2*(path_pts[i+2][0] - 4*path_pts[i+1][0] + 5*path_pts[i][0] - 2*path_pts[i-1][0])
		y_der = 2*(path_pts[i+2][1] - 4*path_pts[i+1][1] + 5*path_pts[i][1] - 2*path_pts[i-1][1])

	elif(i > 1 and i < len(path_pts)-2):
		x_der = 2*(path_pts[i+2][0] - 4*path_pts[i+1][0] + 6*path_pts[i][0] - 4*path_pts[i-1][0] + path_pts[i-2][0])
		y_der = 2*(path_pts[i+2][1] - 4*path_pts[i+1][1] + 6*path_pts[i][1] - 4*path_pts[i-1][1] + path_pts[i-2][1])

	elif(i == len(path_pts)-2):
		x_der = 2*(-2*path_pts[i+1][0] + 5*path_pts[i][0] - 4*path_pts[i-1][0] + path_pts[i-2][0])
		y_der = 2*(-2*path_pts[i+1][1] + 5*path_pts[i][1] - 4*path_pts[i-1][1] + path_pts[i-2][1])


	deriv = [x_der,y_der]
	return np.dot(w_smth,deriv).tolist()


def ortho(pt1,pt2):
	a = np.array(pt1)
	b = np.array(pt2)
	arg = np.dot(np.matmul(np.transpose(a),b),1/(distance(b,[0,0])**2))

	return np.subtract(a, np.dot(arg,b))


def partDer_curvature(i,path_pts):
	global w_curv,kmax

	deriv = []
	# print(i,len(path_pts))

	if(i > 0 and i < len(path_pts)-1):
		x1i = np.array(path_pts[i-1])
		xi = np.array(path_pts[i])
		xi1 = np.array(path_pts[i+1])

		Dxi = xi-x1i												#Dxi
		Dxi1 = xi1-xi												#Dxi1
		num = np.matmul(np.transpose(Dxi),Dxi1)

		mDxi = distance(Dxi,[0,0])									#mod(Dxi)
		mDxi1 = distance(Dxi1,[0,0])								#mod(Dxi1)
		mxi = distance(xi,[0,0])
		mxi1 = distance(xi1,[0,0])

		den = mDxi*mDxi1

		if(mDxi != 0 and mDxi1 != 0 and mxi != 0 and mxi1 != 0):
			if(abs(num/den) < 1):
				Dphi = math.acos(num/den)
				ki = Dphi/mDxi

				if(abs(ki) > kmax):
					arg11 = -1/mDxi 			
					# print("fffff",math.cos(Dphi))							#check why the '-' sign.
					arg12 = -1/math.sqrt(1 - (math.cos(Dphi))**2)

					arg21 = -Dphi/(mDxi**2)
					argi_22 = np.array([1,1])
					arg1i_22 = np.array([-1,-1])

					p1 = np.dot(ortho(xi,-xi1),1/(mxi*mxi1))
					p2 = np.dot(ortho(-xi1,xi),1/(mxi*mxi1))

					arg1i_13 = p2
					argi_13 = -p1 - p2
					argi1_13 = p1

					arg1i = arg11*arg12*arg1i_13 + arg21*arg1i_22
					argi = arg11*arg12*argi_13 + arg21*argi_22
					argi1 = arg11*arg12*argi1_13

					dki = 0.25*arg1i + 0.5*argi + 0.25*argi1

					deriv = 2*(ki - kmax)*dki

				else:
					deriv = np.array([0,0])
			else:
				deriv = np.array([0,0])
		else:
			deriv = np.array([0,0])
	else:
		deriv = np.array([0,0])

	return np.dot(w_curv,deriv).tolist()

def inRo(val):
	return int(round(val))


t1 = time.time()
print("#############################")

img_main_rgb = cv2.imread('MaInmAP.png')
img_main = cv2.imread('MaInmAP.png',0)
# img_main_rgb = cv2.imread('MaInmAP_mINi.png')
# img_main = cv2.imread('MaInmAP_mINi.png',0)
# img_main = cv2.imread('MaInmAP_mINi2.png',0)

inf_stgo = [[90,465],[470,460]]      
init_orien = -np.pi/2
	
# kernel = np.ones((3,3), np.uint8) 
# img = cv2.dilate(img_main.copy(), kernel, iterations=1)

astarObj = Astar_BiCycle(inf_stgo,init_orien,img.copy())
solution,path_waypts,path_pts = astarObj.main_class()
print("aSTar gEneRAtEd")
print("#############################")

brushObj = brushFire(img_main)
mat_dist,mat_ptObs,_ = brushObj.main()
print("bRUsHfiRe gEneRAtEd")
print("---------------------------------------------------")


imgPath = img_main_rgb.copy()
alpha_lr = 0.1
cv2.circle(imgPath,tuple(inf_stgo[0]),3,255,-1)
cv2.circle(imgPath,tuple(inf_stgo[1]),3,255,-1)
for pt in path_pts:
	imgPath[inRo(pt[1])][inRo(pt[0])] = [200,0,0]

path_opt = path_pts[:]

itr = 1
itr_max = 50
pd_coll_lis = []
while(itr <= itr_max):
	path_opt_nxt = []
	imgD = imgPath.copy()

	if(itr != 0 and itr%10 == 0):
		pure_pursObj = pure_pursuit(inf_stgo[0],init_orien,inf_stgo[1],path_opt[:],astarObj.velocity,astarObj.len_veh,10)
		path_opt = pure_pursObj.genPursuit()

	i = 0
	while(i < len(path_opt)):
		pt = path_opt[i]
		pt_iR = [inRo(pt[0]),inRo(pt[1])]	

		pt_obs = mat_ptObs[hash(tuple(pt_iR))]
		pt_obs_dist = mat_dist[pt_iR[1]][pt_iR[0]]

		pd_apf = partDer_apf(pt,pt_obs,pt_obs_dist)
		pd_coll = partDer_obsColl(pt,pt_obs)
		pd_smth = partDer_smth(i,path_opt)
		pd_curv = partDer_curvature(i,path_opt)

		pd_coll_lis.append(pd_coll)	

		pt_new = [pt[0] - alpha_lr*(pd_apf[0] + pd_coll[0] + pd_smth[0] + pd_curv[0]),pt[1] - alpha_lr*(pd_apf[1] + pd_coll[1] + pd_smth[1] + pd_curv[1])]

		path_opt_nxt.append(pt_new)
		imgD[inRo(pt_new[1])][inRo(pt_new[0])] = (0,0,255)

		i = i + 1

	# cv2.imshow('image',imgD)
	# cv2.waitKey(1)

	path_opt = path_opt_nxt[:]
	print('Epoch ' + str(itr))

	itr = itr + 1

t2 = time.time()
print("------Python------")
print("Time : ",t2-t1)

# print(max(pd_coll_lis))
# cv2.imshow('image',imgD)
# cv2.waitKey(0)





# INSIGHTS : 
# Give float values for calculations as any integers could result wrong.
# There's no significance of using Artificial potential fields with other parameters.
# Remember to divide the update term with w_tot.
# The smootheness term tries to keep the points closer as they diverge with iterations.