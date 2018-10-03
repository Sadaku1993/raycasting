import numpy as np
import matplotlib.pyplot as plt
import math

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

    precast = [[] for i in range(int(round((math.pi * 2.0) / yawreso) + 1))]

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

ox = (np.random.rand(10) - 0.5) * grid_size * xy_reso #[m]
oy = (np.random.rand(10) - 0.5) * grid_size * xy_reso #[m]

print("min_x:%6.2f[m] max_x:%6.2f[m] min_y:%6.2f[m] max_y:%6.2f[m]" % 
        (min_x, max_x, min_y, max_y))

pmap = np.zeros((grid_size, grid_size))
precast = precasting(min_x, min_y, grid_size, grid_size, xy_reso, yaw_reso)

for (x, y) in zip(ox, oy):

        d = math.sqrt(x**2 + y**2)
        angle = atan_zero_to_twopi(y, x)
        angleid = math.floor(angle / yaw_reso)

        gridlist = precast[int(angleid)]

        ix = int(round((x - min_x) / xy_reso))
        iy = int(round((y - min_y) / xy_reso))
        
        if (grid_size-1<ix or grid_size-1<iy):
            continue

        for grid in gridlist:
            if grid.d > d:
                pmap[grid.ix][grid.iy] = 0.5

        pmap[ix][iy] = 1.0

px = []
py = []
pmap_grid = np.where(0.5<pmap)
for ix, iy in zip(pmap_grid[0], pmap_grid[1]):
    x = ix*xy_reso + min_x
    y = iy*xy_reso + min_y
    px.append(x)
    py.append(y)
    print(x, y, ix, iy)

plt.cla()
plt.xlim([-grid_size/2*xy_reso, grid_size/2*xy_reso])
plt.ylim([-grid_size/2*xy_reso, grid_size/2*xy_reso])
plt.gca().set_aspect('equal', adjustable='box')
plt.plot(ox, oy, "xr")
draw_line(px, py)
plt.show()

lidar_num = int(round((math.pi * 2.0) / yaw_reso) + 1)
lidar = np.zeros((lidar_num, 4)) # x, y, distance, flag
for ix, iy in zip(pmap_grid[0], pmap_grid[1]):
    x = ix*xy_reso + min_x
    y = iy*xy_reso + min_y
    d = math.sqrt(x**2 + y**2)
    angle = atan_zero_to_twopi(y, x)
    angleid = math.floor(angle / yaw_reso)
    lidar[angleid][0] = x
    lidar[angleid][1] = y
    lidar[angleid][2] = d
    lidar[angleid][3] = True

print(lidar)
