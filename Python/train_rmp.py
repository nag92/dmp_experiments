# -*- coding: utf-8 -*-
"""


@author: nathaniel
adaptived from studywolf: https://studywolf.wordpress.com/2013/11/16/dynamic-movement-primitives-part-1-the-basics/
"""

import numpy as np
import math
from lxml import etree
import scipy.interpolate




def train_rmp(name, n_bfs, T, dt):
    """

    :param name: file name to save too
    :param n_bfs: number of functions
    :param T: trajectory
    :param dt: time step
    :return: None
    """

    def gen_psi(timestep,h, c):
        """
        Generates the activity of the basis functions for a given
        canonical system state or path.

        :param h: Varience of the mean
        :param c: gausain centers
        :return: psi
        """

        x_track = np.zeros(timestep)
        x = 1
        tau = 1

        for t in xrange(timestep):
            x_track[t] = x
            x += 1 * tau * dt

        x_track = x_track[:, None]

        psi = np.exp(h * (np.cos(x_track - c) - 1))

        return psi

    def gen_weights(bfs, psi, f_target):
        """
        Generate a set of weights over the basis functions such
        that the target forcing term trajectory is matched.

        :param bfs: number of bases funtions
        :param psi: activity of the basis functions for a given
        :param f_target: force target
        :return: np.array: the desired forcing term trajectory
        """

        w = np.zeros(bfs)
        for b in xrange(bfs):
            w[b] = (np.dot(psi[:, b], f_target[:]) / (np.sum(psi[:, b]) + 1e-10))

        return w

    def gen_goal(y_des):
        """
        Generate the goal for pattah imitation.
        For rhythmic DMPs the goal is the average of the
        desired trajectory.

        :param y_des: path
        :return: goal of path
        """

        num_idx = ~np.isnan(y_des)  # ignore nan's when calculating goal
        goal = .5 * (y_des[num_idx].min() + y_des[num_idx].max())

        return goal

    def gen_centers(bfs):
        """
        Set the centre of the Gaussian basis
        functions be spaced evenly throughout run time
        :param bfs: number of bases functions
        :return: the centers of the bases function
        """
        c = np.linspace(0, 2 * np.pi, bfs + 1)
        c = c[0:-1]
        return c

    def force(y, yd, ydd, goal):
        """
        calculate the forcing term

        :param y: traj
        :param yd: vel
        :param ydd: accel
        :param goal: goal location
        :return: forcing term
        """
        # constance for dampening

        alpha_z = 25.0
        beta_z = 0.25*alpha_z

        # acclocate memory
        f_target = np.zeros((len(y), 1))

        # create the force target
        for i, (y_i, yd_i, ydd_i) in enumerate(zip(y, yd, ydd)):
            f_target[i] = (ydd_i - alpha_z * (beta_z * (goal - y_i) - yd_i))

        return f_target

    def gen_path(y, timestep, dt):
        """
        generate the path

        :param y: traj
        :param timestep: number of time steps
        :param dt: time step
        :return: pos,vel,accel
        """

        path = np.zeros(timestep)
        x = np.linspace(0, 2 * math.pi, y.shape[1])

        # interplote path
        path_gen = scipy.interpolate.interp1d(x, y)
        for t in range(timestep):
            path[t] = path_gen(t * dt)
        y_des = path

        # calc the vel
        dy_des = np.diff(y_des) / dt
        dy_des = np.hstack((np.zeros((1)), dy_des))

        # calc the accel
        ddy_des = np.diff(dy_des) / dt
        ddy_des = np.hstack((np.zeros((1)), ddy_des))

        return y_des, dy_des, ddy_des

    def save(w, c, h, y0, goal):
        """
        Saves the data to a CSV file so that is can be used by a runner
        :param w: weights from training
        :param c: gausian centers
        :param h: Varience of the mean
        :param y0: startpoint
        :param goal: goal
        :return: None
        """

        w_st = ["%.6f" % number for number in w]
        h_st = ["%.6f" % number for number in h]
        c_st = ["%.6f" % number for number in c]

        root = etree.Element('RMPs')
        weights = etree.Element('Weights')
        inv_sq_var = etree.Element('inv_sq_var')
        gauss_means = etree.Element('gauss_means')
        y_start = etree.Element('y0')
        the_goal = etree.Element('goal')

        the_goal.text = np.str(np.int(goal))
        y_start.text = np.str(np.int(y0))

        for _w, _c, _h in zip(w_st, c_st, h_st):
            etree.SubElement(weights, "w").text = _w
            etree.SubElement(inv_sq_var, "h").text = _h
            etree.SubElement(gauss_means, "c").text = _c

        root.append(weights)
        root.append(inv_sq_var)
        root.append(gauss_means)
        root.append(the_goal)
        root.append(y_start)
        tree = etree.ElementTree(root)
        tree.write(name, pretty_print=True, xml_declaration=True, encoding="utf-8")

    # set up constance

    timestep = int(2 * math.pi / dt)
    y0 = 0
    y, yd, ydd = gen_path(T, timestep, dt)
    goal = gen_goal(T)
    force = force(y, yd, ydd, goal)
    c = gen_centers(n_bfs)
    h = np.ones(n_bfs) * n_bfs  # 1.75
    psi = gen_psi(timestep, h, c)
    w = gen_weights(n_bfs, psi, force)
    save(w, c, h, y0, goal)


path1 = np.sin(np.arange(0, 2 * np.pi, .01) * 5)
T = np.array([path1])
dt = .01

train_rmp("names.xml", 500, T, dt)
