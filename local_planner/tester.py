
#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,TransformStamped

from scipy.spatial import KDTree
import csv
import numpy as np

import pandas as pd
# import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
from sklearn import datasets 
from sklearn.preprocessing import PolynomialFeatures 

def regression(data,degree=3,graph=False):
    X = data[:,0]
    y = data[:,1]
    X=X.reshape(-1,1)
    
    if graph==True:
        plt.scatter(X, y, color = 'blue')  
        plt.xlabel('X') 
        plt.ylabel('Y') 
        plt.show()
    
    poly = PolynomialFeatures(degree = degree) 
    X_poly = poly.fit_transform(X) 
    poly.fit(X_poly, y) 
    lin2 = LinearRegression() 
    lin2.fit(X_poly, y) 
    
    if graph==True:
        plt.scatter(X, y, color = 'blue') 
        plt.plot(X, lin2.predict(poly.fit_transform(X)), color = 'red') 
        plt.title('Polynomial Regression') 
        plt.xlabel('X') 
        plt.ylabel('Y') 
        plt.show()
    
    return lin2,poly,X

def predict(lin2,poly,X_,no_points=100):
    arr=[]
    temp=np.linspace(min(X_), max(X_), num = no_points, endpoint = True, retstep = False, dtype = None)
    for i in range(len(temp)):
        arr.append([temp[i][0],lin2.predict(poly.fit_transform(temp[i][0].reshape(-1,1)))[0]])
    return arr


def pathCallBack(msg,ros_publisher):
    pathLst=[]
    for indx in range(len(msg.poses)):
        pos=msg.poses[indx]
        pathLst.append([pos.pose.position.x,pos.pose.position.y])
    print(pathLst)
    lin2,poly,X = regression(pathLst,degree=2)
    waypt_lst = predict(lin2,poly,X,len(pathLst))
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
     rospy.init_node( 'smoother', anonymous=True)
     ros_publisher = rospy.Publisher('/globalTrajectory2', Path, queue_size=50)  
     rospy.Subscriber('/globalTrajectory', Path, pathCallBack,ros_publisher)
     rospy.spin()

main()