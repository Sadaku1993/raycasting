#coding:utf-8
import numpy as np
import matplotlib.pyplot as plt
import math
from function import calc_angleid, atan_zero_to_twopi

class lidarinfo:
    def __init__(self):
        self.px = 0.0
        self.py = 0.0
        self.d = 0.0
        self.angle = 0.0
        self.angleid = 0
        self.init = False
    def __str__(self):
        return str(self.px) + "," + str(self.py) + "," + str(self.d) + "," + str(self.angle) + "," + str(self.angleid) + "," + str(self.init)

# set pose
pose = np.array([0.0, 0.0, math.radians(90)]) # x, y, yaw

# set resulution
xyreso = 0.50 # x-y grid resolution
yawreso = math.radians(10) # yaw angle resolution [rad]

# set lidar param
min_range = 0.30 # [m]
max_range = 30.0 # [m]
angle_limit = math.radians(270) # [rad]

# grid_map
grid_size = 100
min_x = -grid_size/2
max_z = grid_size/2
min_y = -grid_size/2
max_y = grid_size/2

for i in range(5):
    # init grid_map
    grid_map = np.zeros((grid_size, grid_size))
    # add obstacle
    ox = (np.random.rand(20) - 0.5) * grid_size * xyreso #[m]
    oy = (np.random.rand(20) - 0.5) * grid_size * xyreso #[m]
    
    for x, y in zip(ox, oy):
        ix = int(x/xyreso-min_x)
        iy = int(y/xyreso-min_y)
        grid_map[ix][iy] = 1.0
    
    # get obstacle_grid id
    # この手間を行っている理由はagentを追加するときなどgrid_mapから
    # 障害物位置を導出できた方がいいから
    obstacle_grid = np.where(0<grid_map) # id
    obstacle_position = np.zeros((len(obstacle_grid[0]), 2))
    for i, (ix, iy) in enumerate(zip(obstacle_grid[0], obstacle_grid[1])):
        x = (ix+min_x)*xyreso
        y = (iy+min_y)*xyreso
        obstacle_position[i][0] = x
        obstacle_position[i][1] = y
    # print(obstacle_position)

    # transform
    transform_obstacle = np.zeros((len(obstacle_grid[0]), 2))
    for i in range(len(obstacle_grid[0])):
        transform_obstacle[i][0] = obstacle_position[i][0]-pose[0]
        transform_obstacle[i][1] = obstacle_position[i][1]-pose[1]
    # print(transform_obstacle)

    # rotation
    rotation = np.array([[math.cos(-pose[2]), -math.sin(-pose[2])],
                         [math.sin(-pose[2]),  math.cos(-pose[2])]]);
    rotation_obstacle = np.zeros((len(obstacle_grid[0]), 2))
    for i in range(len(obstacle_grid[0])):
        rotation_obstacle[i] = np.dot(rotation, transform_obstacle[i])
    # print(rotation_obstacle)

    # raycasting
    lidar_num = int(round((math.pi * 2.0) / yawreso) + 1)

    lidar_array = [[] for i in range(lidar_num)]
    for i in range(lidar_num):
        lidar = lidarinfo()
        lidar.angle = i*yawreso
        lidar.angleid = i
        lidar_array[i] = lidar

    for i in range(len(obstacle_grid[0])):
        x = rotation_obstacle[i][0]
        y = rotation_obstacle[i][1]

        d = np.sqrt(x**2+y**2)
        angle = math.atan2(y, x)

        # outof range 
        if(d<min_range or max_range<d):
            continue
        
        # outof angle
        if(angle<-angle_limit/2 or angle_limit/2<angle):
            continue

        angleid = calc_angleid(angle, yawreso)

        if not lidar_array[int(angleid)].init:
            lidar_array[int(angleid)].px = x
            lidar_array[int(angleid)].py = y
            lidar_array[int(angleid)].d  = d
            lidar_array[int(angleid)].angle = angle
            lidar_array[int(angleid)].angleid = angleid
            lidar_array[int(angleid)].init = True
        
        elif d < lidar_array[int(angleid)].d:
            lidar_array[int(angleid)].px = x
            lidar_array[int(angleid)].py = y
            lidar_array[int(angleid)].d  = d

    raycast_x = []
    raycast_y = []
    dumy_x = []
    dumy_y = []

    raycast_map = []

    for lidar in lidar_array:
        angle = lidar.angle
        if math.pi < angle:
            angle -= math.pi*2
        if lidar.init:
            x = lidar.d*math.cos(angle)
            y = lidar.d*math.sin(angle)
            raycast_x.append(x)
            raycast_y.append(y)
            raycast_map.append([x, y])
        elif(-angle_limit/2<=angle and angle<=angle_limit/2):
            x = max_range*math.cos(angle)
            y = max_range*math.sin(angle)
            dumy_x.append(x)
            dumy_y.append(y)
            raycast_map.append([x, y])

    raycast_array = np.array(raycast_map)
    # print(raycast_array)

    # rotation
    inverse = np.array([[math.cos(pose[2]), -math.sin(pose[2])],
                         [math.sin(pose[2]),  math.cos(pose[2])]]);
    raycast_rotation = np.zeros((len(raycast_array), 2))
    for i in range(len(raycast_array)):
        raycast_rotation[i] = np.dot(inverse, raycast_array[i])
    # print(raycast_rotation)

    # transform
    raycast_transform = np.zeros((len(raycast_array), 2))
    for i in range(len(raycast_array)):
        raycast_transform[i][0] = raycast_rotation[i][0]+pose[0]
        raycast_transform[i][1] = raycast_rotation[i][1]+pose[1]

    for x, y in zip (raycast_transform.T[0], raycast_transform.T[1]):
        print("x:%6.2f y:%6.2f" % (x, y))

    fig = plt.figure(figsize=(10,8))
    axL = plt.subplot2grid((2,2), (0,0))
    axR = plt.subplot2grid((2,2), (0,1))
    axLD = plt.subplot2grid((2,2), (1,0))
    axRD = plt.subplot2grid((2,2), (1,1))
    
    axL.plot(ox, oy, "ob")
    axL.set_title('world')
    # axL.set_xlabel('t')
    # axL.set_ylabel('x')
    axL.set_xlim(-grid_size/2*xyreso, grid_size/2*xyreso)
    axL.set_ylim(-grid_size/2*xyreso, grid_size/2*xyreso)
    axL.set_aspect('equal', adjustable='box')
    axL.quiver(pose[0],pose[1], 5*math.cos(pose[2]), 5*math.sin(pose[2]),angles='xy',scale_units='xy',scale=1)
    axL.grid(True)
    
    axR.plot(ox, oy, "ob")
    for x, y in zip(raycast_transform.T[0], raycast_transform.T[1]):
        axR.plot([0.0, x], [0.0, y], 'b-')
    axR.set_title('raycast')
    # axR.set_xlabel('t')
    # axR.set_ylabel('x')
    axR.set_xlim(-grid_size/2*xyreso, grid_size/2*xyreso)
    axR.set_ylim(-grid_size/2*xyreso, grid_size/2*xyreso)
    axR.set_aspect('equal', adjustable='box')
    axR.quiver(pose[0],pose[1], 5*math.cos(pose[2]), 5*math.sin(pose[2]),angles='xy',scale_units='xy',scale=1)
    axR.grid(True)

    axLD.plot(rotation_obstacle.T[0], rotation_obstacle.T[1], "ob")
    axLD.set_title('transform')
    # aDxL.set_xlabel('t')
    # aDxL.set_ylabel('x')
    axLD.set_xlim(-grid_size/2*xyreso, grid_size/2*xyreso)
    axLD.set_ylim(-grid_size/2*xyreso, grid_size/2*xyreso)
    axLD.set_aspect('equal', adjustable='box')
    axLD.quiver(0, 0, 3.0, 0,angles='xy',scale_units='xy',scale=1)
    axLD.grid(True)
 
    for x, y in zip(dumy_x, dumy_y):
        axRD.plot([0.0, x], [0.0, y], 'c-')
    for x, y in zip(raycast_x, raycast_y):
        axRD.plot([0.0, x], [0.0, y], 'b-')
    axRD.plot(rotation_obstacle.T[0], rotation_obstacle.T[1], "ob")
    axRD.set_title('raycast')
    # axR.set_xlabel('t')
    # axR.set_ylabel('x')
    axRD.set_xlim(-grid_size/2*xyreso, grid_size/2*xyreso)
    axRD.set_ylim(-grid_size/2*xyreso, grid_size/2*xyreso)
    axRD.set_aspect('equal', adjustable='box')
    axRD.grid(True)

    fig.show()
    fig.savefig("image.png")

