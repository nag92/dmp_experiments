import xml.etree.ElementTree as ET
import numpy as np
from math import pi
import re


class RMP_runner(object):
    """docstring for RMP_runner
    """

    def __init__(self, file, tau=1, dt=0.01):

        self.tau = tau
        self.dt = 0.01
        self.w = None
        self.c = None
        self.h = None
        self.y0 = None
        self.goal = None

        self.timesteps = int(2 * pi / self.dt)
        self.readInXML(file)

        print self.w

    def gen_psi(self):

        x_track = np.zeros(self.timesteps)
        x = 1
        for t in xrange(self.timesteps):
            x_track[t] = x
            x += 1 * self.tau * self.dt
        x_track = x_track[:, None]

        return np.exp(self.h * (np.cos(x_track - self.c) - 1))

    def gen_front_term(self, x):
        """Generates the front term on the forcing term.
        For rhythmic DMPs it's non-diminishing, so this
        function is just a placeholder to return 1.

        x float: the current value of the canonical system
        dmp_num int: the index of the current dmp
        """

        if isinstance(x, np.ndarray):
            return np.ones(x.shape)
        return 1

    def step(self, tau=1.0, error=0.0, external_force=None):
        """Run the DMP system for a single timestep.

        tau float: scales the timestep
                   increase tau to make the system execute faster
        error float: optional system feedback
        """

        error_coupling = 1.0 / (1.0 + error)
        alpha_z = 25
        beta_z = np.divide(float(alpha_z), 4)
        # run canonical system

        x_track = np.zeros(self.timesteps)
        x = 1
        for t in xrange(self.timesteps):
            x_track[t] = x
            x += 1 * tau * dt
        x_track = x_track[:, None]

        # generate basis function activation
        psi = self.gen_psi(x)


        front_term = self.gen_front_term(x)
        # generate the forcing term
        f = (front_term * (np.dot(psi, self.w)) / np.sum(psi))

        # DMP acceleration
        self.ddy[d] = (self.ay[d] *
                       (self.by[d] * (self.goal[d] - self.y[d]) -
                        self.dy[d] / tau) + f) * tau
        if external_force is not None:
            self.ddy[d] += external_force[d]
        self.dy[d] += self.ddy[d] * tau * self.dt * error_coupling
        self.y[d] += self.dy[d] * self.dt * error_coupling

        return self.y, self.dy, self.ddy


    def readInXML(self, filename):

        root = ET.parse(filename).getroot()
        w = []
        h = []
        c = []

        for weight in root.findall("Weights")[0]:
            w.append(float(weight.text))

        for inv_sq in root.findall("inv_sq_var")[0]:
            h.append(float(inv_sq.text))

        for mean in root.findall("gauss_means")[0]:
            c.append(float(mean.text))

        self.w = np.asarray(w)
        self.c = np.asarray(c)
        self.h = np.asarray(h)
        self.goal = float(root.findall("goal")[0].text)
        self.y0 = float(root.findall("y0")[0].text)


runner = RMP_runner("name.xml")

