import pyrealsense2 as rs
import numpy as np
import cv2
import math

def rot_vec2rot_mat(rx, ry, rz):
    theta = math.sqrt(rx*rx + ry*ry + rz*rz)
    
    ux = rx/theta
    uy = ry/theta
    uz = rz/theta
    
    c = math.cos(theta)
    s = math.sin(theta)
    
    C = 1 - c
    
    R = np.matrix([[ux*ux*C + c, ux*uy*C - uz*s, ux*uz*C + uy*s] , [uy*ux*C + uz*s, uy*uy*C + c, uy*uz*C - ux*s] , [uz*ux*C - uy*s, uz*uy*C + ux*s, uz*uz*C + c]])
    return R
    
def rotm_2_euler(T_2_1):
    print("T21:\t{}\n".format(T_2_1))

    sy = math.sqrt(T_2_1[0,0] * T_2_1[0,0] +  T_2_1[1,0] * T_2_1[1,0] )
    if (sy < 1e-6):
        x = np.arctan2(T_2_1[2,1] , T_2_1[2,2])
        y = np.arctan2(-T_2_1[2,0], sy)
        z = np.arctan2(T_2_1[1,0], T_2_1[0,0])
    else:
        x = np.arctan2(-T_2_1[1,2], T_2_1[1,1])
        y = np.arctan2(-T_2_1[2,0], sy)
        z = 0
	    
    return x,y,z

def theta_x(T_2_1):

    
    
    r_11 = T_2_1[0,0]
    r_12 = T_2_1[0,1]
    r_13 = T_2_1[0,2]
    r_21 = T_2_1[1,0]
    r_22 = T_2_1[1,1]
    r_23 = T_2_1[1,2]
    r_31 = T_2_1[2,0]
    r_32 = T_2_1[2,1]
    r_33 = T_2_1[2,2]
    
    theta_x = np.arctan2(r_32, r_33)
    
    return theta_x
    
    
def theta_y(T_2_1):
    
    r_11 = T_2_1[0,0]
    r_12 = T_2_1[0,1]
    r_13 = T_2_1[0,2]
    r_21 = T_2_1[1,0]
    r_22 = T_2_1[1,1]
    r_23 = T_2_1[1,2]
    r_31 = T_2_1[2,0]
    r_32 = T_2_1[2,1]
    r_33 = T_2_1[2,2]
    
    theta_y = np.arctan2(-r_31, math.sqrt(r_32*r_32 + r_33*r_33))
    
    return theta_y
    
    
def theta_z(T_2_1):
    
    r_11 = T_2_1[0,0]
    r_12 = T_2_1[0,1]
    r_13 = T_2_1[0,2]
    r_21 = T_2_1[1,0]
    r_22 = T_2_1[1,1]
    r_23 = T_2_1[1,2]
    r_31 = T_2_1[2,0]
    r_32 = T_2_1[2,1]
    r_33 = T_2_1[2,2]
    
    theta_z = np.arctan2(r_21, r_11)
    
    return theta_z
