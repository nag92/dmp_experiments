# -*- coding: utf-8 -*-
"""
Created on Wed Nov 01 22:45:30 2017

@author: Onkar
"""
import numpy as np


def y_exp_trajectory(dt):    
    t = list(i for i in np.arange(0,1+dt,dt))
    y = list(np.power(t,2))
    y2 = [0]
    y2 = np.append(y2,np.divide(np.diff(y,1),np.power(dt,1)))
    y3 = [0,0]
    y3 = np.append(y3,np.divide(np.diff(y,2),np.power(dt,2)))
    T = []
    T_rec = []
    T_rec.append(y)
    T.append(y)
    T.append(y2)
    T.append(y3)
    return T

def y_lin_trajectory(dt):
    y = list(i for i in np.arange(0,1+dt,dt))
    y2 = [0]
    y2 = np.append(y2,np.divide(np.diff(y,1),np.power(dt,1)))
    y3 = [0,0]
    y3 = np.append(y3,np.divide(np.diff(y,2),np.power(dt,2)))
    T = []
    T_rec = []
    T_rec.append(y)
    T.append(y)
    T.append(y2)
    T.append(y3)
    return T    

def y_step_trajectory(dt):
    y = []
    for i in range(int(1/dt)/2):
        y.append(0)
    for i in range(int(1/dt)/2):
        y.append(1)
    y.append(1)
    y2 = [0]
    y2 = np.append(y2,np.divide(np.diff(y,1),np.power(dt,1)))
    y3 = [0,0]
    y3 = np.append(y3,np.divide(np.diff(y,2),np.power(dt,2)))
    T = []
    T_rec = []
    T_rec.append(y)
    T.append(y)
    T.append(y2)
    T.append(y3)
    return T   

