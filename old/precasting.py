import numpy as np
import math
import matplotlib.pyplot as plt

class precastDB:

    def __init__(self):
        self.px = 0.0
        self.py = 0.0
        self.d = 0.0
        self.angle = 0.0
        self.ix = 0
        self.iy = 0

    def __str__(self):
        return str(self.px) + "," + str(self.py) + "," + str(self.d) + "," + str(self.angle)


def atan_zero_to_twopi(y, x):
    angle = math.atan2(y, x)
    if angle < 0.0:
        angle += math.pi * 2.0

    return angle

def precasting(minx, miny, xw, yw, xyreso, yawreso):

    precast = [[] for i in range(int(round((math.pi * 2.0) / yawreso)) + 1)]

    for ix in range(xw):
        for iy in range(yw):
            px = ix * xyreso + minx
            py = iy * xyreso + miny

            d = math.sqrt(px**2 + py**2)
            angle = atan_zero_to_twopi(py, px)
            angleid = math.floor(angle / yawreso)

            pc = precastDB()

            pc.px = px
            pc.py = py
            pc.d = d
            pc.ix = ix
            pc.iy = iy
            pc.angle = angle

            precast[int(angleid)].append(pc)

    return precast

def draw_line(ray_x, ray_y):
    for x, y in zip(ray_x, ray_y):
        plt.plot([0.0, x], [0.0, y], 'k-')

grid_size = 100
grid_map = np.zeros((grid_size, grid_size))

xy_reso = 0.50 # m
yaw_reso = math.radians(10) # rad

min_x = -grid_size/2 * xy_reso# [m] 
min_y = -grid_size/2 * xy_reso# [m]
max_x = grid_size/2  * xy_reso# [m]
max_y = grid_size/2  * xy_reso# [m]

pose = np.array([0.0, 0.0, math.radians(0.0)]) 
angle_limit = math.radians(270) # [rad]

ox = (np.random.rand(4) - 0.5) * grid_size #[m] 
oy = (np.random.rand(4) - 0.5) * grid_size #[m]

precast = precasting(min_x, min_y, grid_size, grid_size, xy_reso, yaw_reso)

pmap = [ [0.0 for i in range(grid_size)] for i in range(grid_size)]

ox = (np.random.rand(20)-0.5) * grid_size * xy_reso # [m]
oy = (np.random.rand(20)-0.5) * grid_size * xy_reso # [m]

for (x, y) in zip(ox, oy):
    d = math.sqrt(x**2+y**2)
    angle = atan_zero_to_twopi(y, x)
    angleid = math.floor(angle / yaw_reso)
    gridlist = precast[int(angleid)]

    ix = int(round((x - min_x) / xy_reso))
    iy = int(round((y - min_y) / xy_reso))

    if(ix<0 or grid_size-1<ix or iy<0 or grid_size-1<iy):
        continue

    for grid in gridlist:
        if grid.d > d:
            pmap[grid.ix][grid.iy] = 0.5

    print(ix, iy)
    pmap[ix][iy] = 1.0

precast_map = np.array(pmap)
obstacle_position = np.where(0.5<np.array(pmap))
print(obstacle_position)

ray_x = []
ray_y = []

for ix, iy in zip(obstacle_position[0], obstacle_position[1]):
    px = ix*xy_reso+min_x
    py = iy*xy_reso+min_y
    angle = math.atan2(py, px)
    if(-angle_limit/2<=angle and angle<angle_limit/2):
        ray_x.append(px)
        ray_y.append(py)

for i in [yaw_reso * x for x in range(int(round((math.pi * 2.0) / yaw_reso)) + 1)]:
    if math.pi < i:
        i -= math.pi*2
    print(i)


plt.cla()
plt.xlim([-grid_size/2*xy_reso, grid_size/2*xy_reso])
plt.ylim([-grid_size/2*xy_reso, grid_size/2*xy_reso])
plt.grid()
plt.plot(ox, oy, "xr")
draw_line(ray_x, ray_y)
# plt.plot(pose[0], pose[1], "ob")
plt.quiver(pose[0],pose[1], 5*math.cos(pose[2]), 5*math.sin(pose[2]),angles='xy',scale_units='xy',scale=1)
plt.show()


# angle = math.radians(45)
# angleid = math.floor(angle/yaw_reso)
# 
# gridlist = precast[int(angleid)]
# 
# ox = []
# oy = []
# for grid in gridlist:
#     ox.append(grid.px)
#     oy.append(grid.py)
# 
# plt.cla()
# plt.plot(ox, oy, "ob")
# plt.show()
