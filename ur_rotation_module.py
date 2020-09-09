import pyrealsense2 as rs
import numpy as np
import cv2
import math
from numba import jit


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
    

