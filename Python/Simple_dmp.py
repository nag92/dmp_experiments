# -*- coding: utf-8 -*-
"""
Created on Mon Oct 30 16:48:52 2017

@author: Onkar
"""
import numpy as np
from train_dmp import train_dmp

#Name of the file
name = 'test.xml'

#Set no. of basis functions
n_rfs = 35

#Set the time-step
dt = 0.001

##### TRAJECTORY GENERATOR FOR TRAINING ########
t = list(i for i in np.arange(0,1+dt,dt))
y = list(np.power(t,3))
y2 = [0]
y2 = np.append(y2,np.divide(np.diff(y,1),np.power(dt,1)))
y3 = [0,0]
y3 = np.append(y3,np.divide(np.diff(y,2),np.power(dt,2)))
T = []
T.append(y)
T.append(y2)
T.append(y3)
####### TRAJ END ###############################

#Obtain w, c & D (in that order) from below function
Important_values = train_dmp(name, n_rfs, T, dt)

print Important_values[0],'\n\n\n',Important_values[1],'\n\n\n',Important_values[2]