# -*- coding: utf-8 -*-                         
"""
Created on Tue Jun  4 20:13:31 2019

@author: shank
"""
#import math
#from math import pi
import numpy as np
from numpy.linalg import inv   

# defining the mathematics behind the kalman filter
def filter(x, P):
    for n in range(len(measurements)): 
        
        #time update (prediction of the states)
        x = (A*x)+1*u  #prior estimate 
        P = (A*P*A.transpose())+Q #prior error covariance
        #measurement update
        Zk = np.matrix(measurements[n])  #measurement value
        #H is the measurement matrix, R is the sensor noise matrix
        S = H * P * H.transpose() + R
        #Kalman Gain
        K = P * H.transpose() * inv(S)
        #update estimate state via z
        Y = Zk.transpose()-(H*x)
        X = x+(K*Y)  
        #update error covariance
        P = (I - (K * H)) * P
        
        #print estimates of the future states , kalman gain, and error covariance
        print('X:',X)
        print('P:',P)
        print('K:',K)
 
#control signal       
u = np.matrix([[0.], [0.], [0.], [0.],[0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.]])  
   
dt = 0.1
 #use only the measurement values of x-axis, y-axis and roll and yaw angle
measurements = np.matrix([[3, 1, 2, 5],[2, 3, 2, 5],[3, 1, 2, 5],[3, 1, 2, 5]]) 
print (measurements)
#Initial state
x=np.matrix([[325.9375],
             [-1.5625],
             [5.2958*45.6525],
             [3.4567*45.6525],
             [10.0],
             [0.0],
             [0.0],
             [0.0],
             [0],
             [0],
             [0.0],
             [0.0]])                  #initial position x,y,Wx,Wy.......
I=np.eye(12)
#control vector
B=np.matrix([[0.], [0.], [0.], [0.],[0.], [0.], [0.], [0.],[0.]]) 
#State trasition matrix - here we estimate the states in only two direction and in two rotations
A=np.matrix([[1.0,0.0,0.0,0.0,dt,0.0,0.0,0.0,0.5*dt**2,0.0,0.0,0.0],        
             [0.0,1.0,0.0,0.0,0.0,dt,0.0,0.0,0.0,0.5*dt**2,0.0,0.0],     #only Yaw angle is needed but here both yaw and (pitch or roll) is filtered but roll or pitch will always be 0..
             [0.0,0.0,1.0,0.0,0.0,0.0,dt,0.0,0.0,0.0,0.5*dt**2,0.0],     #in our case
             [0.0,0.0,0.0,1.0,0.0,0.0,0.0,dt,0.0,0.0,0.0,0.5*dt**2],
             [0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,dt,0.0,0.0,0.0],
             [0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,dt,0.0,0.0],
             [0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,dt,0.0],
             [0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,dt],
             [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0],
             [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0],
             [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0],
             [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0]])
#measurement matrix that compromises only the certain measurements that has to be estimated
H=np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0],
             [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0],
             [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0],
             [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0]])
    
#sensor noise noted from the user manual of IMU
rp1=190              #Also noise of gps after certain measurement deviations should be added
rp2=0.3   

#Sensor noise matrix        
R=np.matrix([[rp1, 0.0, 0.0, 0.0],
             [0.0, rp1, 0.0, 0.0],
             [0.0, 0.0, rp1, 0.0],         #Noise of position measurement   (noise added only for the positions and not for the velocities and acceleration)
             [1.0, 0.0, 0.0, rp2]])
    
#Initial covariance matrix  (also for the initial values 1 can be used as it is not certain. eye(12))
P=np.matrix([[0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0],
             [0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0],
             [0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0],
             [0.0, 0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0],
             [0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0],
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.09, 0.0, 0.0, 0.0,0.0,0.0,0.0],
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0],   #angular velocity in X direction (say for 0)
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0],   #angular velocity in Y direction  ''
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05/dt,0.0,0.0,0.0], #Accy. in  X direction
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.09/dt,0.0,0.0], #Accy. in Y direction
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0],     #angular Accy. in Y direction
             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0]])    #angular Accy. in Y direction 

#process noise variance to be assumed as0.1
sv = 0.1 #variance of process noise

#calculation for uncertainity matrix
G = np.matrix([[dt**2],
               [dt**2],
               [dt**2],
               [dt**2],
               [dt],
               [dt],
               [dt],
               [dt],
               [1],
               [1],
               [1],
               [1]])
    
#uncertainity matrix  
Q = G*G.T*sv

filter(x, P)