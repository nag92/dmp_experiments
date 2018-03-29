import Mod_DMP_runner
from train_dmp import train_dmp
import numpy as np
import matplotlib.pyplot as plt
import math


R_halfpi = np.array([[np.cos(np.pi / 2.0), -np.sin(np.pi / 2.0)],
                     [np.sin(np.pi / 2.0), np.cos(np.pi / 2.0)]])
num_obstacles = 5
obstacles = [0.6,0.22]

def avoid_obstacles(y, dy, goal):
    p = np.zeros(2)
    beta = 20.0 / np.pi
    gamma = 250
    for obstacle in obstacles:
        # based on (Hoffmann, 2009)

        # if we're moving
        if np.linalg.norm(dy) > 1e-5:

            # get the angle we're heading in
            phi_dy = -np.arctan2(dy[1], dy[0])
            R_dy = np.array([[np.cos(phi_dy), -np.sin(phi_dy)],
                             [np.sin(phi_dy), np.cos(phi_dy)]])
            # calculate vector to object relative to body

            obj_vec = obstacle - y
            print "vec",obj_vec
            # rotate it by the direction we're going
            obj_vec = np.dot(R_dy, obj_vec)
            # calculate the angle of obj relative to the direction we're going
            phi = np.arctan2(obj_vec[1], obj_vec[0])

            dphi = gamma * phi * np.exp(-beta * abs(phi))
            R = np.dot(R_halfpi, np.outer(obstacle - y, dy))
            pval = -np.nan_to_num(np.dot(R, dy) * dphi)

            # check to see if the distance to the obstacle is further than
            # the distance to the target, if it is, ignore the obstacle
            if np.linalg.norm(obj_vec) > np.linalg.norm(goal - y):
                pval = 0

            p += pval
    return p


class Repulisive_Function():

    def __init__(self, eta, Q_star):

        self.eta = eta
        self.Q_star = Q_star

    def get_U(self,robot_pos,obs_pos):
        """
        :param robot_pos: position of the robot
        :param goal_pos: position of the goal
        :return: engery
        """
        dist = math.sqrt( ( robot_pos[0] - obs_pos[0] ) ** 2 + ( robot_pos[1] - obs_pos[1] ) ** 2 )
        U = 0

        if dist <= self.Q_star:
            U = 0.5 * self.eta * (  (1.0/dist) - (1.0/self.Q_star)  ) ** 2
        else:
            U = 0
        return np.insert(U,2,0,axis=0)


    def get_nabla_U(self, robot_pos, obs_pos):
        """
        :param robot_pos: position of the robot
        :param goal_pos: position of the goal
        :return: engery
        """

        dist = math.sqrt((robot_pos[0] - obs_pos[0])**2 + (robot_pos[1] - obs_pos[1])**2)
        nabla_dist = ( robot_pos - obs_pos )/( dist )
        U = 0

        if dist <= self.Q_star:
            U = self.eta * ((1.0 / self.Q_star) - (1.0 / dist)) * (nabla_dist/(dist*dist))
            print U
        else:
            U = np.array([[0],[0]])

        return U



def y_exp_trajectory(dt):
    t = list(i for i in np.arange(0,1+dt,dt))
    y = list(np.power(t,3))
    print y
    y2 = [0]
    y2 = np.append(y2,np.divide(np.diff(y,1),np.power(dt,1)))
    #y2 = np.gradient(y)
    y3 = [0,0]
    y3 = np.append(y3,np.divide(np.diff(y,2),np.power(dt,2)))
    #y3 = np.gradient(y,2)
    T = []
    T_rec = []
    T_rec.append(y)
    T.append(y)
    T.append(y2)
    T.append(y3)
    return T

def x_exp_trajectory(dt):
    t = list(i for i in np.arange(0,1+dt,dt))
    y = list(np.power(t,1))
    y2 = [0]
    y2 = np.append(y2,np.divide(np.diff(y,1),np.power(dt,1)))
    #y2 = np.gradient(y)
    y3 = [0,0]
    y3 = np.append(y3,np.divide(np.diff(y,2),np.power(dt,2)))
    #y3 = np.gradient(y,2)
    T = []
    T_rec = []
    T_rec.append(y)
    T.append(y)

    T.append(y2)
    T.append(y3)
    return T




dt = 0.001
y = y_exp_trajectory(dt)
x = x_exp_trajectory(dt)
name_y = 'mode_dmp_test_y.xml'
name_x = 'mode_dmp_test_x.xml'
n_rfs = 200
start = 0
goal = 1
#start = 0

Important_values = train_dmp(name_y, n_rfs, y, dt)
Important_values = train_dmp(name_x, n_rfs, x, dt)
#Name of the file

#goal = 1

my_runner_y = Mod_DMP_runner.Mod_DMP_runner(name_y,start,goal)
my_runner_x = Mod_DMP_runner.Mod_DMP_runner(name_x,start,goal)
repluse = Repulisive_Function(10,0.1)

Y = []
X = []
tau = 1
f = [0,0]
for i in np.arange(0,int(tau/dt)+1):

    '''Dynamic change in goal'''
    #new_goal = 2
    #new_flag = 1
    #if i > 0.6*int(tau/dt):
    #    my_runner.setGoal(new_goal,new_flag)
    '''Dynamic change in goal'''


    (x_t, xd_t, xdd_t) = my_runner_x.step(tau, dt,f[0])
    (y_t, yd_t, ydd_t) = my_runner_y.step(tau, dt)
    #print my_runner.x
    #f = avoid_obstacles(np.array([x_t,y_t]),np.array([xd_t,yd_t]) ,my_runner_y.g)
    #print "obs", np.array([[obstacles[0]],[obstacles[1]]])
    f = repluse.get_nabla_U(np.array([[x_t],[y_t]]),np.array([[obstacles[0]],[obstacles[1]]]) )

    print "f", f
    Y.append(y_t)
    X.append(x_t)
    #print temp

time = np.arange(0,tau+dt,dt)

plt.title("2-D DMP demonstration")
plt.xlabel("Time(t)")
plt.ylabel("Position(y)")
#for obstacle in obstacles:
plot_obs, = plt.plot(obstacles[0], obstacles[1], 'rx', mew=3)
plt.plot(time, y[0])
plt.plot(X,Y)
plt._show()
