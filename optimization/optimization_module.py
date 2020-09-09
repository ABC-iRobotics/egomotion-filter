from numba import njit, prange, cuda, float32, config, threading_layer 
import numpy as np
from timeit import default_timer as timer 
import cv2
import egomotion_filter_module as emf
import pyrealsense2 as rs
import numpy as np
import cv2
import realsense_input_module as ri
import realsense_intrinsics_module as intr
import egomotion_filter_module as emf
import matplotlib.pyplot as plt  
import read_robot_states  
import time 
import sys  
import pptk
import ur_tests_module as tests
from mpl_toolkits import mplot3d
from numpy.polynomial.polynomial import polyfit
from enum import Enum
import ur_rotation_module as urrot
from numba import jit
from numba import cuda, float32
import optimization_module as om
import pyrealsense2 as rs
import numpy as np
import cv2
import realsense_input_module as ri
import realsense_intrinsics_module as intr
import egomotion_filter_module as emf
import matplotlib.pyplot as plt  
import read_robot_states  
import time 
import sys  
import pptk
import ur_tests_module as tests
from mpl_toolkits import mplot3d
from numpy.polynomial.polynomial import polyfit
from enum import Enum
import ur_rotation_module as urrot
from numba import jit
from numba import cuda, float32
import optimization_module as om

   
# Point cloud visualization:  https://github.com/heremaps/pptk

class SpeedSource(Enum):
    CONST = 1
    ROBOT = 2
    T265 = 3

# Crop data to cut the acceleration
time_crop_point_start = 5 * 1000.0
time_crop_point_end = 30 * 1000.0

# Velocity camera
velocity_camera = [-0.08/30, 0.0, 0.0]



# Transformation between camera and TCP
zeros_one = np.matrix('0 0 0 1')

R_cam_tcp = np.matrix('-1 0 0; 0 0 -1; 0 -1 0')
t_cam_tcp = np.matrix('-0.025; -0.053; 0.058')


T_cam_tcp = np.append(R_cam_tcp, t_cam_tcp, axis = 1)
T_cam_tcp = np.append(T_cam_tcp, zeros_one, axis = 0)
#print(T_cam_tcp)

R_tcp_cam = R_cam_tcp.transpose()
t_tcp_cam = -1.0 * R_tcp_cam.dot(t_cam_tcp)
T_tcp_cam = np.append(R_tcp_cam, t_tcp_cam, axis = 1)
T_tcp_cam = np.append(T_tcp_cam, zeros_one, axis = 0)
#print(T_tcp_cam)


# Transformation between the two robots

R_base_base_ball = np.matrix('-1 0 0; 0 -1 0; 0 0 1')
t_base_base_ball = np.matrix('-0.6565; -0.0035; 0.0')


T_base_base_ball = np.append(R_base_base_ball, t_base_base_ball, axis = 1)
T_base_base_ball = np.append(T_base_base_ball, zeros_one, axis = 0)

# Camera settings
width_sr300 = 640
height_sr300 = 480
framerate_sr300 = 30

# Algorithm settings
step = 16
threshold = 0.0015
only_the_ball = False
minRadius=40
maxRadius=65 #65 #75
radiusOffset=8
speed_source = SpeedSource.ROBOT


if speed_source == SpeedSource.CONST:
    print("\nSpeed source CONST\n")
elif speed_source == SpeedSource.ROBOT:
    print("\nSpeed source ROBOT\n")
elif speed_source == SpeedSource.T265:
    print("\nSpeed source T265\n")    


# Choose the data folder
if (len(sys.argv)<1 ): 
    print("Usage details: python egomotion_filter:offline_sync_robot_ball.py path")
    sys.exit()

# Read the robot states
robot_states = read_robot_states.read_robot_states(str(sys.argv[1]) + "/" + "robot_states.txt") 
robot_states_ball = read_robot_states.read_robot_states(str(sys.argv[1]) + "/" + "robot_states_ball.txt") 


# Get config for the stream
config_1 = ri.get_config()
config_2 = ri.get_config()

# Get pipeline for the stream
pipeline_1 = ri.get_pipeline()
pipeline_2 = ri.get_pipeline()

# Start the cameras
profile_1 = tests.start_t265(config_1, pipeline_1, "t265.bag")
profile_2 = tests.start_sr300(config_2, pipeline_2, width_sr300, height_sr300, framerate_sr300, "sr300.bag")

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames_sr300 = ri.get_frames(pipeline_2)
        depth_frame = ri.get_depth_frames(frames_sr300)
        color_frame = ri.get_color_frames(frames_sr300)
        frame_number_color = color_frame.get_frame_number()
        frame_number_depth = depth_frame.get_frame_number()
        
       
        # Get the images
        depth_image = ri.convert_img_to_nparray(depth_frame)
        color_image = ri.convert_img_to_nparray(color_frame)
        
        depth_colormap = ri.get_depth_colormap(depth_image, 0.03)
        images = np.hstack((color_image, depth_colormap))
        ri.show_imgs('SR300', images, 1)
        
        # Convert RGB image to gray (for optical flow)
        gray = emf.rgb_to_gray(color_image)

        # Align depth to color
        depth_frame_aligned = emf.align_depth_to_color(frames_sr300)

        # Deprjection
        deprojected = emf.deproject(depth_frame_aligned, 1, 479)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = ri.get_depth_colormap(depth_image, 1)

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))
        
        print(deprojected.shape)
        
    print("Executed succesfully.")

           
except (KeyboardInterrupt, SystemExit):
    print("Program exited.")
    



