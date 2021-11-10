#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,TransformStamped

from scipy.spatial import KDTree
import csv
import numpy as np
# import pandas as pd
# from sklearn.linear_model import LinearRegression
# from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
# from sklearn import datasets 
# from sklearn.preprocessing import PolynomialFeatures 

waypoint_list=[]


with open('/home/fluxauto/imp_ws/GPS_TEST.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        waypoint_list.append([float(row[0]),float(row[1])])

    # waypoint_list = waypoint_list.reverse()

# waypoint_list = waypoint_list[::-1]
# print(waypoint_list)
def get_way_point_tree_list(lane_waypoints):
    re_created_wp=[[w[0],w[1]] for w in lane_waypoints]
    # print(re_created_wp)
    waypoints_tree=KDTree(re_created_wp)
    return waypoints_tree



def movingAverage(num,waypts):
    filteredWpt=[]
    for i in range(len(waypts)):
        tmp = 0.0
        num_tmp = 0.0
        count = 0.0
        if (i - num < 0):
            num_tmp = i 
        elif (i + num > len(waypts) - 1):
            num_tmp = len(waypts) - i - 1
        else:
            num_tmp=num

        for j in range(-num_tmp,num_tmp+1):
            tmp += waypts[i + j]
            count+=1
        filteredWpt.append(tmp / count)
    return filteredWpt

def smooth(path,weight_data=0.5,weight_smooth = 0.1,tolerance = 0.000001):
    newpath=[ [0 for row in range(len(path[0]))] for col in range(len(path))]

    for i in range(len(path)):
        for j in range(len(path[0])):
            newpath[i][j] = path[i][j]

    change = tolerance
    while change >= tolerance:
        change = 0.0
        for i in range(1,len(path)-1):
            for j in range(len(path[0])):
                aux = newpath[i][j]
                newpath[i][j] += weight_data*(path[i][j]-newpath[i][j])
                newpath[i][j] += weight_smooth*(newpath[i-1][j] + newpath[i+1][j]-(2.0 * newpath[i][j]))
                change+=abs(aux-newpath[i][j])
    return newpath

# waypoint_list=smooth(waypoint_list,weight_data=0.5,weight_smooth = 0.38)
x_wpt = [d[0] for d in waypoint_list]
y_wpt = [d[1] for d in waypoint_list]
xAvg = movingAverage(35,x_wpt)
yAvg = movingAverage(35,y_wpt)

x_np = np.array([[d] for d in xAvg])
y_np = np.array([[d] for d in yAvg])
waypoint_list= np.concatenate((x_np,y_np),axis=1)
# X = [d[0] for d in waypoint_list]
# y = [d[1] for d in waypoint_list]
# plt.scatter(X, y, color = 'blue')  
# plt.show()

waypoints_tree=get_way_point_tree_list(waypoint_list)

# def regression(data,degree=3,graph=False):
#     X = data[:,0]
#     y = data[:,1]
#     X=X.reshape(-1,1)
    
#     if graph==True:
#         plt.scatter(X, y, color = 'blue')  
#         plt.xlabel('X') 
#         plt.ylabel('Y') 
#         plt.show()
    
#     poly = PolynomialFeatures(degree = degree) 
#     X_poly = poly.fit_transform(X) 
#     poly.fit(X_poly, y) 
#     lin2 = LinearRegression() 
#     lin2.fit(X_poly, y) 
    
#     if graph==True:
#         plt.scatter(X, y, color = 'blue') 
#         plt.plot(X, lin2.predict(poly.fit_transform(X)), color = 'red') 
#         plt.title('Polynomial Regression') 
#         plt.xlabel('X') 
#         plt.ylabel('Y') 
#         plt.show()
    
#     return lin2,poly,X

# def predict(lin2,poly,X_,no_points=50):
#     arr=[]
#     temp=np.linspace(min(X_), max(X_), num = no_points, endpoint = True, retstep = False, dtype = None)
#     for i in range(len(temp)):
#         arr.append([temp[i][0],lin2.predict(poly.fit_transform(temp[i][0].reshape(-1,1)))[0]])
#     return arr





def closest_index(localized_point):
  lane_waypoints=waypoint_list
  # print(lane_waypoints[0])
  # waypoints_tree=get_way_point_tree_list(waypoint_list) 
  closest_idx=waypoints_tree.query(localized_point,1)[1]
  closest_coord=lane_waypoints[closest_idx][:2]
  prev_coord=lane_waypoints[closest_idx-1][:2]
  cl_vect=np.array(closest_coord)
  prev_vect=np.array(prev_coord)
  pos_vect=np.array(localized_point)
  val= np.dot(cl_vect-prev_vect,pos_vect-cl_vect)
  if val>0:
    temp=closest_idx
    closest_idx=(closest_idx+1)%len(lane_waypoints)
    if(closest_idx==0 and temp>0):
      closest_idx=temp
  return closest_idx,lane_waypoints[closest_idx:closest_idx+40]   #22

# get_way_point_tree_list(waypoint_list)

def feederCallBack(msg,ros_publisher):
    localized_point =[msg.pose.position.x, msg.pose.position.y]
    index,waypt_lst = closest_index(localized_point)
    # #SMOOTHENING
    # dataWP =np.array(waypt_lst) 
    # # #get first 25 rows
    # # dataWP=dataWP[index:50]
    # lin2,poly,X_=regression(dataWP,degree=2,graph=False)
    # waypt_lst=predict(lin2,poly,X_,no_points=len(dataWP))

    # X = [d[0] for d in waypt_lst]
    # y = [d[1] for d in waypt_lst]
    # plt.scatter(X, y, color = 'blue')  
    # plt.show()

    path_pub=Path()
    path_pub.header.frame_id='world'
    for i in range(len(waypt_lst)):#
        
        new_pose=PoseStamped()
        path_now=waypt_lst[i]
        # quaternion_map=tf.transformations.quaternion_from_euler(0.0, 0.0, 0)
        new_pose.header.seq=i+1
        new_pose.header.frame_id='world'
        new_pose.pose.position.x=path_now[0]
        new_pose.pose.position.y=path_now[1]
        #print(new_pose.pose.position.x,new_pose.pose.position.y)
        new_pose.pose.position.z=0
        new_pose.pose.orientation.x= 0
        new_pose.pose.orientation.y= 0
        new_pose.pose.orientation.z= 0
        new_pose.pose.orientation.w= 1
        # new_pose.pose.orientation=path_now[3]
        path_pub.poses.append(new_pose)
        # rate = rospy.Rate(50)
        # start_time=time.time() 
        #mpc_pub.publish(path_pub
    ros_publisher.publish(path_pub)





def main():
     rospy.init_node( 'feeder', anonymous=True)
     ros_publisher = rospy.Publisher('/planning/localPlanner/globalTrajectory', Path, queue_size=50)  
     rospy.Subscriber('/ndt_pose', PoseStamped, feederCallBack,ros_publisher)
     rospy.spin()

main()
