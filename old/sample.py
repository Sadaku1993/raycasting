#coding:utf-8

import numpy as np
import matplotlib.pyplot as plt
import math

xy_reso = 0.25 #[m/cell]
yaw_reso = math.radians(10.0) #[rad]

grid_size = 100 #[cell]

robot_radius = 0.5    # [m]
obstacle_radius = 0.5 # [m]

def is_collision(pose, ox, oy):
    flag = False
    for x,y in zip(ox, oy):
        distance = np.sqrt((x-pose[0])**2+(y-pose[1])**2)
        if distance<robot_radius+obstacle_radius:
            print("collision at x:%5.2f y:%5.2f d:%5.2f" % (x, y, distance)) 
            flag = True
            break
    return flag

def atan_zero_to_twopi(y, x):
    angle = math.atan2(y, x)
    if angle < 0.0:
        angle += math.pi * 2.0

    return angle

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

def precasting(grid_map, xyreso, yawreso):
    print('precast size:', round((math.pi * 2.0 / yawreso) + 1))

    precast = [ [] for i in range(int( round((math.pi*2.0/yawreso)+1) ))]

    for ix in range(grid_size):
        for iy in range(grid_size):
            px = (ix - grid_size/2) * xyreso #[m]
            py = (iy - grid_size/2) * xyreso #[m]
            d = math.sqrt(px**2 + py**2)
            angle = atan_zero_to_twopi(py, px)
            angleid = math.floor(angle/yawreso)
            pc = precastDB()
            pc.px = px
            pc.py = py
            pc.d = d
            pc.ix = ix
            pc.iy = iy
            pc.angle = angle
            precast[int(angleid)].append(pc)
    return precast
    
for i in range(2):
    # map
    grid_map = np.zeros((grid_size, grid_size))
    
    # x, y, yaw(-180~180deg)
    pose = np.array([0.0, 0.0, math.radians(0.0)]) 
    angle_limit = math.radians(270) # [rad]
    angle_centroid = pose[2]
    
    ox = (np.random.rand(10)-0.5) * grid_size * xy_reso # [m]
    oy = (np.random.rand(10)-0.5) * grid_size * xy_reso # [m]

    print("x:%5.2f y:%5.2f yaw:%5.2f max_angle%6.2f min_angle:%6.2f" %
           (pose[0], pose[1],pose[2], angle_limit/2, -angle_limit/2))
    
    flag = is_collision(pose, ox, oy)
    if flag:
        continue
    
    for x, y in zip(ox, oy):
        grid_map[x, y] = 1
    
    # robot座標系にて行う
    pmap = [[0.0 for i in range(grid_size)] for i in range(grid_size)]
    precast = precasting(grid_map, xy_reso, yaw_reso)

    for (x, y) in zip(ox, oy):
        # 座標変換
        rx = x * np.cos(pose[2]) - y * np.sin(pose[2])
        ry = x * np.sin(pose[2]) + y * np.cos(pose[2])
        tx = rx + pose[0]
        ty = ry + pose[1]
        d = math.sqrt(tx**2+ty**2)
        angle = math.atan2(ty, tx) # (-pi ~ pi)
        angle_dis = abs(angle-angle_centroid)
        if(angle_limit/2<angle_dis):
            print("out of range x:%6.2f y:%6.2f angle:%6.2f" % (x, y, angle))
            continue
        if angle<0.0:
            angle += math.pi * 2.0
        angleid = math.floor(angle/yaw_reso)
        gridlist = precast[int(angleid)]

        # gridのどこに存在するか
        ix = int(round(x/xy_reso+grid_size/2))
        iy = int(round(y/xy_reso+grid_size/2))

        print("ix:%d iy:%d" % (ix, iy))

        for grid in gridlist:
            if grid.d > d:
                pmap[grid.ix][grid.iy] = 0.5
        pmap[ix][iy] = 2.0

    angleid = math.floor(math.radians(45)/yaw_reso)
    print angleid
    for grid in gridlist:
        print(grid)

    obstacle_grid = np.where(1.0<=pmap)
    print(obstacle_grid)


        
    plt.xlim([-grid_size/2*xy_reso, grid_size/2*xy_reso])
    plt.ylim([-grid_size/2*xy_reso, grid_size/2*xy_reso])
    plt.grid()
    plt.plot(ox, oy, "xr")
    # plt.plot(pose[0], pose[1], "ob")
    plt.quiver(pose[0],pose[1], 5*math.cos(pose[2]), 5*math.sin(pose[2]),angles='xy',scale_units='xy',scale=1)
    plt.show()
