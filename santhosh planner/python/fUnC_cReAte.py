import numpy as np 
import math

pts_path = []

def distance(pt1,pt2):                             #Euclidean Distance
	return np.sqrt((pt2[0]-pt1[0])*(pt2[0]-pt1[0]) + (pt2[1]-pt1[1])*(pt2[1]-pt1[1]))

def cost_at_eachPt(pts):
	
	pts_und_Dist = []

	dist = 0
	prev = pts[-1]
	i = len(pts) - 1
	while(i >= 0):
		dist = dist + distance(prev,pts[i])
		pts_und_Dist.append([pts[i],dist])
		prev = pts[i]
		i = i - 1

	pts_und_Dist.reverse()

	return pts_und_Dist

def pt_inORout(pt,pts):
	if(round(distance(pt,pts[0]) + distance(pt,pts[1]) - distance(pts[0],pts[1])) == 0):
		return True
	else:
		return False

def perpend_drop(pt,pts):
	[x1,y1] = pts[0]
	[x2,y2] = pts[1]
	[x3,y3] = pt

 	k = ((y2-y1) * (x3-x1) - (x2-x1) * (y3-y1))/((y2-y1)**2 + (x2-x1)**2)
	x4 = x3 - k * (y2-y1)
	y4 = y3 + k * (x2-x1)

	return [x4,y4], distance([x4,y4],pt), pt_inORout([x4,y4],pts)

def finNeaPt(pt,pts):

	dist = float("inf")
	i = 0
	j = -1
	while(i < len(pts)):
		if(distance(pt,pts[i]) < dist):
			dist = distance(pt,pts[i])
			j = i
		i = i + 1

	return j

def finHeuPt(pt,pts):

	ind_clsPt = finNeaPt(pt, pts)
	if(ind_clsPt != len(pts)):
		pt_drop,dist_drop,bool_drop = perpend_drop(pt,[pts[ind_clsPt],pts[ind_clsPt+1]])

		if(bool_drop == True):
			return ind_clsPt+1
		else:
			return ind_clsPt
	else:
		return ind_clsPt

def count_ninetys(a1,a2):
	ninetys = [-np.pi,-np.pi/2,0,np.pi/2,np.pi,3*np.pi/2,2*np.pi,5*np.pi/2,3*np.pi,7*np.pi/2,4*np.pi]
	ninetys_in = []

	for ang in ninetys:
		if(a1 < ang and ang < a2):
			ninetys_in.append(ang)

	# print("Ninety Angles : ",np.dot(ninetys_in,180/np.pi).tolist())
	return ninetys_in

def count_end_pts(loc1,loc2,alpha,centre):

	len_veh = 2/math.sqrt(3)
	velocity = 1
	time_step = 1
	pts_x_y = []

	if(round(alpha*180/np.pi) != 0):
		radi = len_veh/math.tan(alpha)
		time_rev = 2*np.pi*abs(radi)/velocity

		if(time_step < time_rev):
			pts_x_y = [[loc1[0],loc2[0]],[loc1[1],loc2[1]]]
			ang1 = math.atan2(loc1[1] - centre[1],loc1[0] - centre[0])
			ang2 = math.atan2(loc2[1] - centre[1],loc2[0] - centre[0])

			if(math.tan(alpha) > 0):
				if(ang2 < ang1):
					ang2 = ang2 + 2*np.pi

				angs_in = count_ninetys(ang1,ang2)
				for nine_ang in angs_in:
					pts_x_y[0].append(centre[0] + abs(radi)*math.cos(nine_ang))
					pts_x_y[1].append(centre[1] + abs(radi)*math.sin(nine_ang))

			else:
				if(ang1 < ang2):
					ang1 = ang1 + 2*np.pi

				angs_in = count_ninetys(ang2,ang1)
				for nine_ang in angs_in:
					pts_x_y[0].append(centre[0] + abs(radi)*math.cos(nine_ang))
					pts_x_y[1].append(centre[1] + abs(radi)*math.sin(nine_ang))

		else:
			pts_x_y = [[centre[0] - abs(radi),centre[0] + abs(radi),centre[0],centre[0]],[centre[1],centre[1],centre[1] - abs(radi),centre[1] + abs(radi)]]

	else:
		pts_x_y = [[loc1[0],loc2[0]],[loc1[1],loc2[1]]]

	return pts_x_y

def disp_action(img,loc1,loc2,alpha,centre):
	len_veh = 2/math.sqrt(3)
	velocity = 1
	time_step = 1

	if(round(alpha*180/np.pi) != 0):
		radi = len_veh/math.tan(alpha)
		time_rev = 2*np.pi*abs(radi)/velocity

		if(time_step < time_rev):
			ang1 = math.atan2(loc1[1] - centre[1],loc1[0] - centre[0])
			ang2 = math.atan2(loc2[1] - centre[1],loc2[0] - centre[0]) 

			if(math.tan(alpha) > 0):
				if(ang2 < ang1):
					ang2 = ang2 + 2*np.pi
			else:
				if(ang2 > ang1):
					ang1 = ang1 + 2*np.pi
			cv2.ellipse(img,(int(centre[0]),int(centre[1])),(int(abs(radi)),int(abs(radi))),0,ang1*180/np.pi,ang2*180/np.pi,255,1)

		else:
			cv2.circle(img,(int(centre[0]),int(centre[1])),int(abs(radi)),255,1)

		cv2.circle(img,(int(loc1[0]),int(loc1[1])),2,155,-1)
		cv2.circle(img,(int(loc2[0]),int(loc2[1])),2,155,-1)

	else:
		cv2.line(img,(int(loc1[0]),int(loc1[1])),(int(loc2[0]),int(loc2[1])),255,1)
		cv2.circle(img,(int(loc2[0]),int(loc2[1])),2,155,-1)


def obstacle_check(node1,node2,alpha,centre):
	
	end_pts = count_end_pts(node1,node2,alpha,centre)

	x_list = end_pts[0]
	y_list = end_pts[1]

	x_mi = int(np.amin(x_list)) - 3
	if(x_mi < 0):
		x_mi = 0

	x_ma = int(np.amax(x_list)) + 3
	if(x_ma >= self.img_w):
		x_ma = self.img_w - 1

	y_mi = int(np.amin(y_list)) - 3
	if(y_mi < 0):
		y_mi = 0

	y_ma = int(np.amax(y_list)) + 3
	if(y_ma >= self.img_h):
		y_ma = self.img_h - 1


	img_obst_roi = self.img[y_mi:y_ma,x_mi:x_ma]

	origin_roi = [x_mi,y_mi]

	node1_new = [node1[0] - origin_roi[0],node1[1] - origin_roi[1]]
	node2_new = [node2[0] - origin_roi[0],node2[1] - origin_roi[1]]
	centre_new = [centre[0] - origin_roi[0],centre[1] - origin_roi[1]]

	win_action = np.zeros(img_obs_roi.shape,np.uint8)
	disp_action(win_action,node1_new,node2_new,alpha,centre_new)

	obs_coll = cv2.bitwise_and(win_action,img_obs_roi)
	if(np.amax(obs_coll) > 10):
		return True

	elif(node2[0] < 0 or node2[0] >= self.img_w or node2[1] < 0 or node2[1] >= self.img_h):
		return True

	else:
		return False





t1 = 2*np.pi/3
t2 = 5*np.pi/6
loc1 = [2*math.cos(t1),2*math.sin(t1)]
loc2 = [2*math.cos(t2),2*math.sin(t2)]
alpha = -np.pi/6
centre = [0,0]

ptsXY = count_end_pts(loc1,loc2,alpha,centre)
print("Points")
i = 0
while(i < len(ptsXY[0])):
	print(ptsXY[0][i],ptsXY[1][i])
	i = i + 1
