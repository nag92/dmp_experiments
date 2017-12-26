# -*- coding: utf-8 -*-
"""


@author: nathaniel
adpatived from studywolf: 
"""
# name       : filename for trained dmp to be stored into
# n_rfs      : number of basis functions to train dmp with
# T          : training trajectory formatted: [Y; dY; ddY];
# dt         : timestep between each sequential trajectory point

import numpy as np
import math
from lxml import etree
import scipy.interpolate


def train_rmp(name, n_bfs, T, dt):
    
    name = name
    y = T
    dt = dt

    # Extract trajectory into pos, vel, accl vectors

    #the time constants for chosen for critical damping
    alpha_z = 25
    beta_z  = np.divide(float(alpha_z),4)
    alpha_g = np.divide(float(alpha_z),2)
    alpha_v = alpha_z
    beta_v  = beta_z
    timesteps = int( 2*math.pi / dt)
    tau = 1


    def gen_psi(h,c):
        """Generates the activity of the basis functions for a given
        canonical system state or path.

        x float, array: the canonical system state or path
        """

        x_track = np.zeros(timesteps)
        x = 1
        for t in xrange(timesteps):
            x_track[t] = x
            x += 1*tau*dt
        x_track = x_track[:, None]
        
        return np.exp(h * (np.cos(x_track - c) - 1))

    def gen_weights(psi,f_target):
        """Generate a set of weights over the basis functions such
        that the target forcing term trajectory is matched.

        f_target np.array: the desired forcing term trajectory
        """
        w = np.zeros(n_bfs)
        for b in range(n_bfs):
            w[b] = (np.dot(psi[:, b], f_target[:]) / (np.sum(psi[:, b]) + 1e-10))

        return w    

    def gen_goal(y_des):
        """Generate the goal for pattah imitation.
        For rhythmic DMPs the goal is the average of the
        desired trajectory.

        y_des np.array: the desired trajectory to follow
        """

        num_idx = ~np.isnan(y_des)  # ignore nan's when calculating goal
        goal = .5 * (y_des[num_idx].min() + y_des[num_idx].max())

        return goal

    def gen_centers():
        """Set the centre of the Gaussian basis
        functions be spaced evenly throughout run time"""

        c = np.linspace(0, 2*np.pi, n_bfs+1)
        c = c[0:-1]
        return c

    def force(y,yd,ydd, goal):
        """
        """
        
        f_target = np.zeros((len(y), 1))
        # find the force required to move along this trajectory
        
        for i, (y_i, yd_i, ydd_i) in  enumerate( zip( y,yd,ydd  ) ): 
            f_target[ i ] = ( ydd_i - alpha_z * (beta_z *  (goal - y_i) - yd_i))
        return f_target


    def gen_path():

        path = np.zeros(timesteps)
        x = np.linspace(0, 2*math.pi, y.shape[1])

        path_gen = scipy.interpolate.interp1d(x, y)
        for t in range(timesteps):
            path[t] = path_gen(t *dt)
        y_des = path

        dy_des = np.diff(y_des) / dt
            # add zero to the beginning of every row
        dy_des = np.hstack((np.zeros((1)), dy_des))

        # calculate acceleration of y_des
        ddy_des = np.diff(dy_des) / dt
        # add zero to the beginning of every row
        ddy_des = np.hstack((np.zeros((1)), ddy_des))
        return y_des, dy_des, ddy_des

    def save(h,c,w):

        root = etree.Element('DMPs')
        weights = etree.Element('Weights')
        inv_sq_var = etree.Element('inv_sq_var')
        gauss_means = etree.Element('gauss_means')
        dGx = etree.Element('dG')
        dGx.text = np.str(np.int(dG))
        Ax = etree.Element('A')
        Ax.text = np.str(np.int(A))
        sx = etree.Element('s')
        sx.text = str(np.int(s))
        y0x = etree.Element('y0')
        y0x.text = np.str(np.int(y0))


    for i in range(0,n_rfs):
        etree.SubElement(weights, "w").text = w_st[i]
        etree.SubElement(inv_sq_var, "D").text = D_st[i]
        etree.SubElement(gauss_means, "c").text = c_st[i]


    

    # set variance of Gaussian basis functions
    # trial and error to find this spacing
    
    y,yd,ydd = gen_path()   
    goal = gen_goal(T)
    force = force(y,yd,ydd,goal)
    c = gen_centers()
    h = np.ones(n_bfs) * n_bfs  # 1.75
    psi = gen_psi(h,c)
    w = gen_weights(psi,force)



path1 = np.sin(np.arange(0, 2*np.pi, .01)*5)
T = np.array([path1])
dt = .01

train_rmp("name", 100,T, dt)
