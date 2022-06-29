from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
import numpy as np

f = open('drawing_traj.txt', 'r')
lines = f.readlines()
xs = []; ys = []; zs =[]

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.set_box_aspect([1,1,1])

for line in lines:
    if line == "End\n":
        ax.plot(xs,ys,zs, color='black')
        # plt.ion()
        # plt.draw()
        xs = []; ys = []; zs =[]
    else :
        line = line.split()
        xs.append(float(line[0]))
        ys.append(float(line[1]))
        zs.append(float(line[2]))

ax.set_xlim3d([0.75, 0.95])
ax.set_ylim3d([-0.1, 0.1])
ax.set_zlim3d([0.35, 0.55])
plt.show()
