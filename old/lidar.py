#coding:utf-8
import numpy as np
import matplotlib.pyplot as plt
import math

def atan_zero_to_twopi(y, x):
    angle = math.atan2(y, x)
    if angle < 0.0:
        angle += math.pi * 2.0

    return angle

def calc_angleid(angle):
    if angle < 0.0:
        angle += math.pi * 2.0
    angleid = math.floor(angle / yaw_reso)
    return angleid

def draw_line(ray_x, ray_y):
    for x, y in zip(ray_x, ray_y):
        plt.plot([0.0, x], [0.0, y], 'k-')

def dumy_line(ray_x, ray_y):
    for x, y in zip(ray_x, ray_y):
        plt.plot([0.0, x], [0.0, y], "b-")

grid_size = 100
grid_map = np.zeros((grid_size, grid_size))

xy_reso = 0.50 # m
yaw_reso = math.radians(10) # rad

min_x = -grid_size/2 * xy_reso# [m] 
min_y = -grid_size/2 * xy_reso# [m]
max_x = grid_size/2  * xy_reso# [m]
max_y = grid_size/2  * xy_reso# [m]

min_range = 0.1  # [m]
max_range = 30.0 # [m]

pose = np.array([0.0, 0.0, math.radians(0.0)]) 
angle_limit = math.radians(270) # [rad]

ox = (np.random.rand(20) - 0.5) * grid_size * xy_reso #[m]
oy = (np.random.rand(20) - 0.5) * grid_size * xy_reso #[m]

lidar_num = int(round((math.pi * 2.0) / yaw_reso) + 1)
lidar_array = np.zeros((lidar_num, 6)) # x, y, distance, angle, init, id

for x, y in zip(ox, oy):
    d = np.sqrt(x**2+y**2)   # [m]
    angle = math.atan2(y, x) # -pi~pi[rad]
    
    # outof range 
    if(d<min_range or max_range<d):
        continue
    # outof angle
    if(angle<-angle_limit/2 or angle_limit/2<angle):
        continue

    angleid = calc_angleid(angle)

    if not lidar_array[angleid][4]:
        lidar_array[angleid][0] = x
        lidar_array[angleid][1] = y
        lidar_array[angleid][2] = d
        lidar_array[angleid][3] = angle
        lidar_array[angleid][4] = 1
        lidar_array[angleid][5] = angleid

    elif d<lidar_array[angleid][2]:
        lidar_array[angleid][0] = x
        lidar_array[angleid][1] = y
        lidar_array[angleid][2] = d
        lidar_array[angleid][3] = angle

dumy_ray_x = []
dumy_ray_y = []
for angleid in range(lidar_num):
    angle = angleid * yaw_reso
    if math.pi < angle:
        angle -= math.pi*2
    if angle < -angle_limit/2 or angle_limit/2 < angle:
        continue

    if not lidar_array[angleid][4]:
        dumy_ray_x.append(max_range*math.cos(angle))
        dumy_ray_y.append(max_range*math.sin(angle))

ray_x = []
ray_y = []

for lidar in lidar_array:
    if not lidar[4]:
        continue
    print(lidar)
    angle = lidar[5] * yaw_reso
    if math.pi < angle:
        angle -= math.pi*2
    ray_x.append(lidar[2]*math.cos(angle))
    ray_y.append(lidar[2]*math.sin(angle))
    # ray_x.append(lidar[0])
    # ray_y.append(lidar[1])

plt.cla()
plt.plot(ox, oy, "ob")
draw_line(ray_x, ray_y)
dumy_line(dumy_ray_x, dumy_ray_y)
plt.show()
