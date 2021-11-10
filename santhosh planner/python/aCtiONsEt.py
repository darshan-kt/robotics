import cv2
import numpy as np 
import math


# img = np.zeros((500,500),np.uint8)
# # cv2.line(img,(256,256),(356,256),255,1)
# cv2.ellipse(img,(1109, 250), (859, 859), 0, -180.0, -173.33265632324205, 255, 1)
# cv2.imshow('image',img)
# cv2.waitKey(0)


img = np.zeros((500,500),np.uint8)
alpha = -np.pi/6
# alpha = np.pi/180
loc = [250,250]
theta1 = np.pi/2
velocity = 10
time_step = 10
locs_actSet = []
leng = 15
i = 0

while(alpha <= np.pi/6):
	if(round(alpha*180/np.pi) != 0):
		radi = leng/math.tan(alpha)
		time_rev = 2*np.pi*abs(radi)/velocity

		theta2 = theta1 + (velocity*math.tan(alpha)/leng)*(time_step)
		loc_new = [loc[0] + (leng/math.tan(alpha))*(math.sin(theta2) - math.sin(theta1)), loc[1] + (leng/math.tan(alpha))*(math.cos(theta1) - math.cos(theta2))]

		centre_botFram = [0,radi]
		theta_trans = -theta1
		loc_trans = [-loc[0],-loc[1]]
		centre_global = [radi*math.sin(theta_trans) - loc_trans[0], radi*math.cos(theta_trans) - loc_trans[1]]

		if(time_step < time_rev):
			ang1 = math.atan2(loc[1] - centre_global[1],loc[0] - centre_global[0])
			ang2 = math.atan2(loc_new[1] - centre_global[1],loc_new[0] - centre_global[0]) 

			if(math.tan(alpha) > 0):
				if(ang2 < ang1):
					ang2 = ang2 + 2*np.pi
			else:
				if(ang2 > ang1):
					ang1 = ang1 + 2*np.pi
			cv2.ellipse(img,(int(centre_global[0]),int(centre_global[1])),(int(abs(radi)),int(abs(radi))),0,ang1*180/np.pi,ang2*180/np.pi,255,1)

		else:
			cv2.circle(img,(int(centre_global[0]),int(centre_global[1])),int(abs(radi)),255,1)

		cv2.circle(img,(int(loc[0]),int(loc[1])),2,155,-1)
		cv2.circle(img,(int(loc_new[0]),int(loc_new[1])),2,155,-1)

	else:
		loc_new = [loc[0] + velocity*math.cos(theta1)*time_step,loc[1] + velocity*math.sin(theta1)*time_step]
		theta2 = theta1
		cv2.line(img,(int(loc[0]),int(loc[1])),(int(loc_new[0]),int(loc_new[1])),255,1)
		cv2.circle(img,(int(loc_new[0]),int(loc_new[1])),2,155,-1)

	alpha = alpha + np.pi/180

cv2.imshow('image',img)
cv2.waitKey(100)
