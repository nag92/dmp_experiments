import train_rmp
import RMP_runner
import numpy as np
import matplotlib.pyplot as plt


path1 = np.sin(np.arange(0, 2 * np.pi, .01) * 5)
T = np.array([path1])
dt = .01

train_rmp.train_rmp("names.xml", 100, T, dt)
runner = RMP_runner.RMP_runner("names.xml")
# y_track = np.zeros(runner.timesteps)
# dy_track = np.zeros(runner.timesteps)
# ddy_track = np.zeros(runner.timesteps)
#
# for t in xrange(runner.timesteps):
#     y_track[t],dy_track[t],ddy_track[t] = runner.step()

y,dy,ddy = runner.run()
print y

plt.plot(y)
plt.plot(path1)
plt.show()
# print "hello"