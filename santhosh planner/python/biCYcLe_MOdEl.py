import cv2
import numpy as np 
import math


def bicycle_model(orientation, alpha, bot_loc, velocity, time_step, bot_leng):

	radi = bot_leng/math.tan(alpha)
	orientation_next = orientation + (velocity*math.tan(alpha)/bot_leng)*(time_step)
	bot_loc_new = [bot_loc[0] + (bot_leng/math.tan(alpha))*(math.sin(orientation_next) - math.sin(orientation)), bot_loc[1] + (bot_leng/math.tan(alpha))*(math.cos(orientation) - math.cos(orientation_next))]

	return [bot_loc_new,orientation_next]


orientation = -np.pi/2
alpha = np.pi/6
bot_loc = [250,250]
velocity = 20
time_jump = 0.1
time_step_max = 5
bot_leng = 30
img = np.zeros((500,500),np.uint8)


while(round(alpha*180/np.pi) >= -30):
	print('Alpha : ' + str(round(alpha*180/np.pi)))

	time_step = 0
	if(round(alpha*180/np.pi) != 0):
		while(time_step < time_step_max):
			loc, orien = bicycle_model(orientation,alpha,bot_loc,velocity,time_step, bot_leng)
			loc_int = [int(loc[0]),int(loc[1])]
			img[loc_int[1]][loc_int[0]] = 255
			
			print('Time : ' + str(time_step) + ', Location : ' + str(loc_int) + ', Orientation : ' + str(round(orien*180/np.pi)))
			time_step = time_step + time_jump

	else:
		while(time_step < time_step_max):
			loc = [bot_loc[0] + velocity*math.cos(orientation)*time_step,bot_loc[1] + velocity*math.sin(orientation)*time_step]
			loc_int = [int(loc[0]),int(loc[1])]
			img[loc_int[1]][loc_int[0]] = 255

			print('Time : ' + str(time_step) + ', Location : ' + str(loc_int) + ', Orientation : ' + str(round(orien*180/np.pi)))
			time_step = time_step + time_jump

	print('##########################################################')
	print('##########################################################')

	cv2.imshow('image',img)
	key = cv2.waitKey(1)
	if(key == 27):
		break
	elif(key == 32):
		cv2.waitKey(0)


	alpha = alpha - np.pi/180


cv2.waitKey(0)