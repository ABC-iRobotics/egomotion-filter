import pyrealsense2 as rs
import numpy as np
import cv2
import realsense_input_module as ri
import egomotion_filter_module as emf
import matplotlib.pyplot as plt  
import read_robot_states  
import time 
import sys  
import pptk
import read_robot_states

        
        

def start_sr300(config, pipeline, width_sr300, height_sr300, framerate_sr300, filename):
    # Get the camera data from a bag file
    rs.config.enable_device_from_file(config, str(sys.argv[1]) + "/" + filename)

    # Enable stream
    config.enable_stream(rs.stream.depth, width_sr300, height_sr300, rs.format.z16, framerate_sr300)
    config.enable_stream(rs.stream.color, width_sr300, height_sr300, rs.format.bgr8, framerate_sr300)

    # Start pipeline
    profile_2=pipeline.start(config)
    
    # Playback from bag
    playback_2 = profile_2.get_device().as_playback()
    playback_2.set_real_time(False)
    
    return profile_2

def start_t265(config, pipeline, filename):

    # Get the camera data from a bag file
    rs.config.enable_device_from_file(config, str(sys.argv[1]) + "/" + filename)

    # Enable stream
    config.enable_stream(rs.stream.pose)

    # Start pipeline
    profile_1 = pipeline.start(config)
    
    # Playback from bag
    playback_1 = profile_1.get_device().as_playback()
    playback_1.set_real_time(False)
    
    return profile_1
    

def velocity_ball_to_camera_frame(velcity_ball, T_camera_tcp, T_base_base_ball, robot_state):
    homegenous_velocity_ball = np.append(np.asmatrix(velcity_ball), np.matrix('0'), axis = 1).transpose()
    homogenous_velocity_ball_cf = np.asarray(\
        T_camera_tcp.dot(robot_state.T_tcp_base).dot( \
        T_base_base_ball).dot(homegenous_velocity_ball))
    velocity_cf= homogenous_velocity_ball_cf[0:3,:].flatten()
    return velocity_cf































