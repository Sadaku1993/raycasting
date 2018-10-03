#coding:utf-8

import numpy as np
import matplotlib.pyplot as plt
import math

def atan_zero_to_twopi(y, x):
    angle = math.atan2(y, x)
    if angle < 0.0:
        angle += math.pi * 2.0
    return angle

def calc_angleid(angle, yaw_reso):
    if angle < -yaw_reso/2:
        angle += math.pi * 2.0
    trans_angle = angle + yaw_reso/2
    angleid = math.floor(trans_angle/yaw_reso)
    return angleid

def draw_line(ray_x, ray_y, color):
    for x, y in zip(ray_x, ray_y):
        plt.plot([0.0, x], [0.0, y], color)

grid_size = 100

xy_reso = 0.5 # [m]
yaw_reso = math.radians(10) # [rad]

min_range = 0.1  # [m]
max_range = 30.0 # [m]
angle_limit = math.radians(270) # [rad]

ox = (np.random.rand(20) - 0.5) * grid_size * xy_reso #[m]
oy = (np.random.rand(20) - 0.5) * grid_size * xy_reso #[m]

lidar_num = int(round((math.pi * 2.0) / yaw_reso) + 1)
lidar_array = np.zeros((lidar_num, 6)) # x, y, distance, angle, id, init

# calc angle
for i in range(lidar_num):
    lidar_array[i][3] = i*yaw_reso

for x, y in zip(ox, oy):
    d = np.sqrt(x**2+y**2)
    angle = math.atan2(y, x)

    # outof range 
    if(d<min_range or max_range<d):
        continue
    # outof angle
    if(angle<-angle_limit/2 or angle_limit/2<angle):
        continue

    angleid = calc_angleid(angle, yaw_reso)

    if not lidar_array[angleid][5]:
        lidar_array[angleid][0] = x
        lidar_array[angleid][1] = y
        lidar_array[angleid][2] = d
        lidar_array[angleid][4] = angleid
        lidar_array[angleid][5] = 1 

    elif d<lidar_array[angleid][2]:
        lidar_array[angleid][0] = x
        lidar_array[angleid][1] = y
        lidar_array[angleid][2] = d

ray_x = []
ray_y = []
for lidar in lidar_array:
    if not lidar[5]:
        continue
    print(lidar)
    angle = lidar[3]
    if math.pi < angle:
        angle -= math.pi*2
    ray_x.append(lidar[2]*math.cos(angle))
    ray_y.append(lidar[2]*math.sin(angle))

dumy_ray_x = []
dumy_ray_y = []
for lidar in lidar_array:
    if lidar[5]:
        continue
    
    angle = lidar[3]
    if math.pi < angle:
        angle -= math.pi*2
    if (angle<-angle_limit/2 or angle_limit/2<angle):
        continue

    dumy_ray_x.append(max_range*math.cos(angle))
    dumy_ray_y.append(max_range*math.sin(angle))

plt.cla()
plt.plot(ox, oy, "ob")
draw_line(ray_x, ray_y, 'b-')
draw_line(dumy_ray_x, dumy_ray_y, 'c-')
plt.show()
