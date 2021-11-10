rospy.init_node("dummy", anonymous=True)

nullFlag = False
BZD = 0
currentVelocity = 5
fusionData = np.zeros((0,3))
workFlag = True
speedFlag = False
emergencyFlag = False

def fuzzyLogic(minima, currentVelocity):
    
    # try:
    global BZD
    BZD = (currentVelocity * currentVelocity)  / (2 * const.DECELARATION_RATE) #Braking Distance at -7 m/s^2  formula
    print(currentVelocity, BZD)
    timeGap = minima[const.POSITION_X] / currentVelocity  
    velocityToFollow = 0

    if timeGap <= const.MAINTAIN_TIME and minima[const.RVY_INDEX] <= 0:
        
        velocityToFollow = 0
        print('Vehicle too close and too slow')

    elif timeGap > const.MAINTAIN_TIME and minima[const.RVY_INDEX] <= 0:
        
        if minima[const.POSITION_X] > BZD:
        
            velocityToFollow = minima[const.RVY_INDEX] + currentVelocity
            print('Vehicle slower but at a distance')
        
        else:
        
            velocityToFollow = minima[const.RVY_INDEX] - abs((minima[const.POSITION_X] - BZD) / const.TA1) + currentVelocity
            print('Vehicle is slower and closer')
            print('Reducing speed by a factor of:', abs((minima[const.POSITION_X] - BZD) / const.TA1), 'to get into safe zone in', TA1, 'seconds')
            print(velocityToFollow)

    elif timeGap <= const.MAINTAIN_TIME and minima[const.RVY_INDEX] > 0:
    
        velocityToFollow = minima[const.RVY_INDEX] - abs((minima[const.POSITION_X] - BZD) / const.TA2) + currentVelocity
        print('Vehicle is faster but closer')
        print('Reducing speed by a factor of:', abs((minima[const.POSITION_X] - BZD) / const.TA2), 'to get into safe zone in', const.TA2, 'seconds')
        print(velocityToFollow)
    else:
    
        if minima[const.RVY_INDEX] < const.MAX_SPEED:
    
            velocityToFollow = minima[const.RVY_INDEX] + currentVelocity
            print('Vehicle faster and farther away')
            print(velocityToFollow)
    
        else:
    
            velocityToFollow = const.MAX_SPEED
            print('Vehicle too fast thus limiting to max speed')

    return velocityToFollow


def minExtrack(fusionData):
	global nullFlag
	filtered = np.zeros((0, fusionData.shape[1]))
	y_thres = const.VEHICLE_WIDTH / const.GRID_RESOLUTION   #y-axis thresold for apply ACC

	for vehicle in fusionData:
		if (abs(vehicle[const.POSITION_Y]) - const.VEHICLE_WIDTH) < (abs(const.Y_FACTOR * y_thres) / 2): #consider the vehicles comes in throsold
			if (vehicle[const.POSITON_X] >= 0):  #consider the vehicles in front of it(ie, +ve x)
				filtered = np.vstack([filtered, vehicle])  #add in vertical stack

	try:
		minimalIndex = np.argmin(filtered, axis = 0) #find the nearest minimum index nums
		minima = filtered[minimalIndex[1]]  # among those consider the only one object and assign that obj info to minima
		nullFlag = False

	except:
		minima = np.zeros(fusionData.shape[1])
		nullFlag = True

	fusionData = np.zeros(0,3)
	print(minima)
	return minima


def subCall(data):
	global fusionData, emergencyFlag, workFlag, speedFlag
	fusedData = fused_data
	fusionData = np.zeros((0,3))

	stopCount = 0
	turnCount = 0
	pedeCount = 0

	for ele in data.fused_data_:

		if ele.Class == "Car" or ele.Class == "Truck":
			tmp = np.array(ele.radX, ele.radY, ele.vels)
			fusionData = np.vstack([fusionData, tmp])
		else:
			if ele.Class == "Person":
				if ele.radX < (BZD + 7) and abs(radY) < 2:
					pedeCount +=1
					emergencyFlag = True
					pubACC.publish(True)

			elif ele.Class == "Stop sign":
				stopCount +=1
				workFlag = False

			elif ele.CLass == "Direction boards right_turn" or ele.Class == "Direction boards left_turn":
				turnCount +=1
				speedFlag = True

	if pedeCount == 0:
		pubACC.publish(False)
		emergencyFlag = False
	if turnCount == 0;
		speedFlag = False

def subOBD(data):
	global currentVelocity
	currentVelocity = data.data


rospy.Subscrber("/fused_data", fused_data_, subCall)
rospy.Subscrber("/OBD", Float64, subOBD)   #coming from embedded
pubACC = rospy.Publisher("/planning/gideon/emergencyFlag", Bool, queue_size = 1)
pubPID = rospy.Publisher("/planning/gideon/VelocityToFollow", Float64, queue_size = 1)

pubACC.publish(False)

while not rospy.is_shutdown():
	print("workFlag:", workFlag, "speedFlag:", speedFlag, "emergencyFlag:", emergencyFlag)
	time.sleep(1)

	if workFlag:
		minima = minExtract(fusionData)

		if not nullFlag:
			print("The nearest extracted vehicle by sensor fusion data is: ", minima)
			print("Vehicle is at a distance of:", minima[0], "meters against a braking distance of ", BZD, "meters")

			VelocityToFollow = fuzzyLogic(minima, 10)
			if velocityToFollow > const.MAX_SPEED:
                velocityToFollow = const.MAX_SPEED
            print('')
            print('The longitudinal relative velocity of the corresponding vehicle is:', minima[2], 'm/s')
            print("currentVelocity is:", currentVelocity)
            print('Speed Limit is:', const.MAX_SPEED, 'm/s')
            print('The velocity to follow is:', velocityToFollow, 'm/s')
            # pubPID.publish(velocityToFollow)
        else:
            print('')
            print('No Vehicle Data obtained')
            print('Travelling at max speed.')
            print('')
            velocityToFollow = const.MAX_SPEED
       
        if speedFlag:
            if velocityToFollow > 7:
                velocityToFollow = 7        

        pubPID.publish(velocityToFollow)
        print('')
        print('*******************************************************************************************************************************')
        print('')
        fusionData = np.zeros((0, 3))

tmp = 50
print('bye...')
pubPID.publish('50')