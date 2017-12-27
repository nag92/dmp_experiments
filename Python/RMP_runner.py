import xml.etree.ElementTree as ET
import numpy as np
from math import pi
import matplotlib.pyplot as plt
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
        self.y = np.zeros(1)
        self.dy = np.zeros(1)
        self.ddy = np.zeros(1)
        self.x = 1
        self.timesteps = int(2 * pi / self.dt)
        self.readInXML(file)



    def gen_psi(self):

        return np.exp(self.h * (np.cos(self.x - self.c) - 1))

    def gen_front_term(self):
        """Generates the front term on the forcing term.
        For rhythmic DMPs it's non-diminishing, so this
        function is just a placeholder to return 1.

        x float: the current value of the canonical system
        dmp_num int: the index of the current dmp
        """

        if isinstance(self.x, np.ndarray):
            return np.ones(self.x.shape)
        return 1

    def step(self, tau=1.0):
        """Run the DMP system for a single timestep.

        tau float: scales the timestep
                   increase tau to make the system execute faster
        error float: optional system feedback
        """


        alpha_z = 25
        beta_z = 0.25*alpha_z
        # run canonical system


        self.x += 1 * self.tau * self.dt

        # generate basis function activation
        psi = self.gen_psi()



        front_term = self.gen_front_term()
        # generate the forcing term
        f = (front_term * (np.dot(psi, self.w)) / np.sum(psi))
        print f
        # DMP acceleration
        self.ddy = (alpha_z *
                       (beta_z * (self.goal - self.y) -
                        self.dy / tau) + f) * tau

        self.dy += self.ddy * tau * self.dt
        self.y  += self.dy * self.dt

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
        self.y = float(root.findall("y0")[0].text)


runner = RMP_runner("names.xml")
y_track = np.zeros(runner.timesteps)
dy_track = np.zeros(runner.timesteps)
ddy_track = np.zeros(runner.timesteps)

for t in xrange(runner.timesteps):
    y_track[t],dy_track[t],ddy_track[t] =     runner.step()
print "y_track", + y_track
plt.plot(y_track)
plt.show()
# print "hello"

