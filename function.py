import numpy as np
import math

def is_collision(pose, ox, oy, robot_radius, obstacle_radius):
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

def calc_angleid(angle, yaw_reso):
    if angle < -yaw_reso/2:
        angle += math.pi * 2.0
    trans_angle = angle + yaw_reso/2
    angleid = math.floor(trans_angle/yaw_reso)
    return angleid
