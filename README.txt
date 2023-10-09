# egomotion-filter

## Overview

Optical flow is an established tool for motion detection in the visual scene. While optical flow algorithms
usually provide accurate results, they can not make a difference between image-space displacements origi-
nated from moving objects in the space and the ego-motion of the moving viewpoint. In the case of optical
flow-based moving object segmentation, camera ego-motion compensation is essential. Hereby, we show the
preliminary results of a moving viewpoint optical flow ego-motion filtering method, using two dimensional op-
tical flow, image depth information and the camera holder robot arm’s state of motion. We tested its accuracy
through physical experiments, where the camera was fixed on a robot arm, and a test object was attached onto
an other robot arm. The test object and the camera were moved relative to each other along given trajectories
in different scenarios. We validated our method for optical flow background filtering, which showed 94.88%
mean accuracy in the different test cases. Furthermore, we tested the proposed algorithm for moving object
state of motion estimation, which showed high accuracy in the case of translational and rotational movements
without depth variation, but lower accuracy, when the relative motion produced change in depth, or the camera
and the moving object move in the same directions. The proposed method with future work including outlier
filtering and optimisation could become useful in various robot navigation applications and optical flow-based
computer vision problems




Install the requirements (in the downloaded folder): pip install -r requirements.txt
Place the data inside the folder.
Run the main with the following: python egomotion_filter_offline_sync_robot_ball.py y_x/poster
(Or with other data)


