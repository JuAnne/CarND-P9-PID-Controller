import matplotlib.pyplot as plt
import numpy as np
import sys

"""
x1 = np.loadtxt('cte_steer.out')
plt.plot(x1[:,0], label = 'CTE', color = 'b')
plt.plot(x1[:,1], label = 'Steer', color = 'r')
"""
x1 = np.loadtxt('pid.out')
plt.plot(-0.06*x1[:,0], label = 'P Controller', color = 'b')
plt.plot(-0.06*x1[:,0] - 1.3*x1[:,2], label = 'PD Controller', color = 'r')
plt.plot(-0.06*x1[:,0] - 1.3*x1[:,2] - 0.0003*x1[:,1], label = 'PID Controller', color = 'y')

plt.xlabel('Iteration')
plt.ylabel('PID')
plt.legend()
plt.show()