

"""
File to determine the maximum accelerations of the robot.

1. Log data and drive robot at full speed.
   $ rosbag record odom cmd_vel
2. Convert to csv file:
   $ rostopic echo -b 2014-12-01-14-44-27.bag -p /odom/twist > measurement.txt
3. Run this file :)
"""

import numpy as np
from scipy.signal import butter, lfilter
import matplotlib.pyplot as plt

# Load data:
data = np.genfromtxt('measurement.txt', skip_header=1, delimiter=',',
    usecols=(0, 1, 6), unpack=True) # dtype=(int, float, float))
t, v, omega = data

# Normalize time:
t = t / 1e9
t = t - t[0]

# Determine fs:
dt = np.mean(np.diff(t))
print('Dt = {}'.format(dt))

# Filter data:
#b, a = butter(4, 0.1, 'low', analog=True)
#omega = lfilter(b, a, omega)

# Plot omega:
plt.figure(1)
plt.subplot(2, 1, 1)
plt.plot(t, omega)
plt.xlabel('Time [s]')
plt.ylabel('rad/s')
plt.grid(True)
plt.subplot(2, 1, 2)
plt.plot(t[:-1], np.diff(omega)/dt)
plt.ylabel('rad/s2')
plt.xlabel('Time [s]')
plt.grid(True)

plt.figure(2)
plt.plot(np.diff(t))

# Plot v:
plt.figure(3)
plt.subplot(2, 1, 1)
plt.plot(t, v)
plt.xlabel('Time [s]')
plt.ylabel('m/s')
plt.grid(True)
plt.subplot(2, 1, 2)
plt.plot(t[:-1], np.diff(v)/dt)
plt.ylabel('m/s2')
plt.xlabel('Time [s]')
plt.grid(True)

plt.show()

