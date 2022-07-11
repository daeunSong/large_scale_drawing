from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.pyplot as plt
import numpy as np

### plot trajectory
f = open('drawing_traj.txt', 'r')
lines = f.readlines()
f.close()
xs = []; ys = []; zs =[]

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.set_box_aspect([1,1,1])
ax.set_xlabel('$X$')
ax.set_ylabel('$Y$')
ax.set_zlabel('$Z$')
ax.grid(True)
strokes = []
stroke = []

for line in lines:
    if line == "End\n":
        ax.plot(xs,ys,zs, color='b')#, linewidth=1.0)
        # plt.ion()
        # plt.draw()
        strokes.append(stroke)
        xs = []; ys = []; zs =[]; stroke = []
    else :
        line = line.split()
        xs.append(float(line[0]))
        ys.append(float(line[1]))
        zs.append(float(line[2]))
        stroke.append([float(line[0]),float(line[1]),float(line[2])])

ax.set_xlim3d([0.75, 0.95])
ax.set_ylim3d([-0.1, 0.1])
ax.set_zlim3d([0.35, 0.55])
# plt.show()

# # plot input drawing path
# f = open('../demo/grid_half_sphere_k_demo.txt', 'r')
# lines = f.readlines()
# xs = []; ys = []; zs =[]
#
# for line in lines[1:]:
#     if line == "End\n":
#         ax.plot(xs,ys,zs, color='k')#, linewidth=1.0)
#         # plt.ion()
#         # plt.draw()
#         xs = []; ys = []; zs =[]
#     else :
#         line = line.split()
#         [x,y,z] = [float(el) for el in line[0:3]]
#         p = np.array([[x],[y],[z], [0]])
#         T = np.array([[0, 0, -1, 0],[0, 1, 0, 0],[1, 0, 0, 1]])
#         p = np.dot(T, p)
#         x = p[0][0]*0.01+0.91003
#         y = p[1][0]*0.01-0.01106
#         z = p[2][0]*0.01+0.445994
#
#         xs.append(x)
#         ys.append(y)
#         zs.append(z)



### degree
# num, start, add order
save = [ [[0, 10, 5], [8, 10, 5]],
  [[1, 22, 3], [9, 19, 3]],
  [[2, 29, 3], [10, 30, 3]],
  [[3, 63, 3], [13, 40, 3]],
  [[2, 72, 2], [14, 29, 2]],
  [[0, 71, 2], [15, 13, 2]],
  [[7, 67, 2], [15, 72, 2]],
  [[3, 52, 2], [12, 40, 2]],
  [[4, 42, 2], [11, 50, 2]],
  [[5, 31, 2], [10, 60, 2]]
  ]

thetas = []
for el in save:
    num1 = el[0][0]
    start1 = el[0][1]
    end1 = start1+el[0][2]
    num2 = el[1][0]
    start2 = el[1][1]
    end2 = start2+el[1][2]

    u = np.array(strokes[num1][end1])-np.array(strokes[num1][start1])
    v = np.array(strokes[num2][end2])-np.array(strokes[num2][start2])
    theta = np.arccos(np.dot(u,v)/(np.linalg.norm(u)*np.linalg.norm(v)))
    thetas.append(theta)

    # ax.plot([strokes[num1][start1][0],strokes[num1][end1][0]],[strokes[num1][start1][1],strokes[num1][end1][1]],[strokes[num1][start1][2],strokes[num1][end1][2]], color='r')#, linewidth=1.0)
    # ax.plot([strokes[num2][start2][0],strokes[num2][end2][0]],[strokes[num2][start2][1],strokes[num2][end2][1]],[strokes[num2][start2][2],strokes[num2][end2][2]], color='r')#, linewidth=1.0)

errs = 0
for theta in thetas:
    err = abs(1.5708 - theta)
    errs = errs + err
print(errs/10.0, "radian")

# plt.show()



### distances
# num, start, add order
save = [ [[0, 10, 10], [8, 10, 10]],
  [[1, 22, 9], [9, 19, 9]],
  [[2, 32, 10], [10, 30, 10]],
  [[3, 62, 10], [13, 40, 10]],
  [[2, 62, 9], [14, 29, 9]],
  [[0, 64, 9], [15, 13, 9]],
  [[7, 61, 8], [15, 66, 9]],
  [[3, 52, 10], [12, 40, 10]],
  [[4, 41, 10], [11, 50, 10]],
  [[5, 31, 9], [10, 60, 9]]
  ]

add = 10
# num1 = 5; start1 = 31; end1 = start1+add
# num2 = 10; start2 = 60; end2 = start2+add
errs = []

for el in save:
    num1 = el[0][0]
    start1 = el[0][1]
    add1 = el[0][2]
    end1 = start1+add1
    num2 = el[1][0]
    start2 = el[1][1]
    add2 = el[1][2]
    end2 = start2+add2

    u = np.array(strokes[num1][end1])-np.array(strokes[num1][start1])
    v = np.array(strokes[num2][end2])-np.array(strokes[num2][start2])

    dist1 = 0
    dist2 = 0

    for i in range(add1):
        p1 = np.array(strokes[num1][start1+i]); p2 = np.array(strokes[num1][start1+i+1])
        dist1 = dist1 + np.linalg.norm(p1-p2)
        ax.plot([p1[0], p2[0]],[p1[1], p2[1]],[p1[2], p2[2]], color='r')#, linewidth=1.0)

    for i in range(add2):
        p3 = np.array(strokes[num2][start2+i]); p4 = np.array(strokes[num2][start2+i+1])
        dist2 = dist2 + np.linalg.norm(p3-p4)
        ax.plot([p3[0], p4[0]],[p3[1], p4[1]],[p3[2], p4[2]], color='r')#, linewidth=1.0)

    err = abs(1.0 - dist1/dist2)
    # print(err)
    errs.append(err)

print(np.average(errs))

plt.show()


