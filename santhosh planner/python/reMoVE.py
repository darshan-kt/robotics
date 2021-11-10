import cv2
import numpy as np 
from numpy import unravel_index
import math 
import time
import csv


w_curv = 1
kmax = 0

def distance(pt1,pt2):
	return math.sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2)

def ortho2(pt1,pt2):
	a = np.array(pt1)
	b = np.array(pt2)
	arg = np.dot(np.matmul(np.transpose(a),b),1/(distance(b,[0,0])**2))

	return np.subtract(a, np.dot(arg,b))


def partDer_curvature2(i,path_pts):
	global w_curv,kmax

	deriv = []

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
		Dphi = math.acos(num/den)
		ki = Dphi/mDxi

		if(abs(ki) > kmax):
			arg11 = -1/mDxi 										#check why the '-' sign.
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

	return np.dot(w_curv,deriv).tolist()


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

		print("Dxi : ",Dxi)
		print("Dxi1 : ",Dxi1)
		print("Num : ",num)
		den = mDxi*mDxi1
		print("mxi : ",mxi)
		print("mxi1 : ",mxi1)
		print("Den : ",den)

		if(mDxi != 0 and mDxi1 != 0 and mxi != 0 and mxi1 != 0):
			if(abs(num/den) < 1):
				Dphi = math.acos(num/den)
				ki = Dphi/mDxi
				print("Dphi : ",Dphi)
				print("ki : ",ki)

				if(abs(ki) > kmax):
					arg11 = -1/mDxi 			
					# print("fffff",math.cos(Dphi))							#check why the '-' sign.
					arg12 = -1/math.sqrt(1 - (math.cos(Dphi))**2)

					arg21 = -Dphi/(mDxi**2)
					argi_22 = np.array([1,1])
					arg1i_22 = np.array([-1,-1])

					p1 = np.dot(ortho(xi,-xi1),1/(mxi*mxi1))
					p2 = np.dot(ortho(-xi1,xi),1/(mxi*mxi1))

					print("p1 : ",p1)
					print("p2 : ",p2)

					arg1i_13 = p2
					argi_13 = -p1 - p2
					argi1_13 = p1

					arg1i = arg11*arg12*arg1i_13 + arg21*arg1i_22
					argi = arg11*arg12*argi_13 + arg21*argi_22
					argi1 = arg11*arg12*argi1_13
					dki = 0.25*arg1i + 0.5*argi + 0.25*argi1

					deriv = 2*(ki - kmax)*dki

					print("1i_1 : ",arg11*arg12*arg1i_13," ,1i_2 : ",arg21*arg1i_22)
					print("i_1 : ",arg11*arg12*argi_13," ,i_2 : ",arg21*argi_22)
					print("i1_1 : ",arg11*arg12*argi1_13)

				else:
					print("1")
					deriv = np.array([0,0])
			else:
				print("2")
				deriv = np.array([0,0])
		else:
			print("3")
			deriv = np.array([0,0])
	else:
		print("4")
		deriv = np.array([0,0])

	print("#######################")
	return np.dot(w_curv,deriv).tolist()



# Test the code and check the cases with 0 in denominator.


pts = [[160.81342194084974, 252.31013125630204], [161.0588835610912, 251.79003882928745], [161.28082846700315, 251.27988484684545], [161.17868746285382, 251.21121714843255], [161.3103083635104, 250.65342148628594], [161.42984051058642, 250.09559367022615], [161.53426860306297, 249.54303532431646]]
print(partDer_curvature(3,pts))














