# The following are the backup functions for various functions in the GlobPlan algorithms.



	def cost_at_eachPt(self,pts):		
		pts_und_Dist = []

		dist = 0
		prev = pts[-1]

		i = len(pts) - 1
		while(i >= 0):
			dist = dist + self.distance(prev,pts[i])
			pts_und_Dist.append([pts[i],dist])
			prev = pts[i]
			i = i - 1

		pts_und_Dist.reverse()
		return pts_und_Dist

	def pt_inORout(self,pt,pts):
		if(round(self.distance(pt,pts[0]) + self.distance(pt,pts[1]) - self.distance(pts[0],pts[1])) == 0):
			return True
		else:
			return False

	def perpend_drop(self,pt,pts):
		[x1,y1] = pts[0]
		[x2,y2] = pts[1]
		[x3,y3] = pt

	 	k = ((y2-y1) * (x3-x1) - (x2-x1) * (y3-y1))/((y2-y1)**2 + (x2-x1)**2)
		x4 = x3 - k * (y2-y1)
		y4 = y3 + k * (x2-x1)

		return [x4,y4], self.distance([x4,y4],pt), self.pt_inORout([x4,y4],pts)

	def finNeaPt(self,pt,pts):
		dist = float("inf")
		i = 0
		j = -1
		while(i < len(pts)):
			if(self.distance(pt,pts[i]) < dist):
				dist = self.distance(pt,pts[i])
				j = i
			i = i + 1

		return j

	def finHeuPt(self,pt):
		pts = self.path_heu_pts
		ind_clsPt = self.finNeaPt(pt, pts)
		if(ind_clsPt < len(pts)-1):
			pt_drop,dist_drop,bool_drop = self.perpend_drop(pt,[pts[ind_clsPt],pts[ind_clsPt+1]])

			if(bool_drop == True):
				return ind_clsPt+1
			else:
				return ind_clsPt
		else:
			return ind_clsPt


	def disp_path(self,img_disp,node1,theta1,alpha):
		len_veh = self.len_veh
		velocity = self.velocity
		time_step = self.time_step
		img_map = self.img.copy()

		n = 30
		time_jump = float(time_step)/n

		i = 0
		while(i <= n):
			theta2 = theta1 + (velocity*math.tan(alpha)/len_veh)*(time_jump)
			if(alpha != 0):
				node2 = [node1[0] + (len_veh/math.tan(alpha))*(math.sin(theta2) - math.sin(theta1)), node1[1] + (len_veh/math.tan(alpha))*(math.cos(theta1) - math.cos(theta2))]
			else:
				node2 = [node1[0] + velocity*math.cos(theta1)*time_jump,node1[1] + velocity*math.sin(theta1)*time_jump]

			if(self.inRo(node2[0]) >= 0 and self.inRo(node2[0]) < img_map.shape[1] and self.inRo(node2[1]) >= 0 and self.inRo(node2[1]) < img_map.shape[0]):
				img_disp[self.inRo(node2[1])][self.inRo(node2[0])] = 125
	
			node1 = node2
			theta1 = theta2
			i = i + 1

		return img_disp

	def obstacle_check2(self,node1,theta1,alpha):
		len_veh = self.len_veh
		velocity = self.velocity
		time_step = self.time_step
		img_map = self.img.copy()

		n = 30
		time_jump = float(time_step)/n

		i = 0
		while(i <= n):
			theta2 = theta1 + (velocity*math.tan(alpha)/len_veh)*(time_jump)
			if(alpha != 0):
				node2 = [node1[0] + (len_veh/math.tan(alpha))*(math.sin(theta2) - math.sin(theta1)), node1[1] + (len_veh/math.tan(alpha))*(math.cos(theta1) - math.cos(theta2))]
			else:
				node2 = [node1[0] + velocity*math.cos(theta1)*time_jump,node1[1] + velocity*math.sin(theta1)*time_jump]

			if(self.inRo(node2[0]) >= 0 and self.inRo(node2[0]) < img_map.shape[1] and self.inRo(node2[1]) >= 0 and self.inRo(node2[1]) < img_map.shape[0]):
				if(img_map[self.inRo(node2[1])][self.inRo(node2[0])] == 255):
					return True
			else:
				return True

			node1 = node2
			theta1 = theta2
			i = i + 1

		return False

