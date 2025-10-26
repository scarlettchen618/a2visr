#!/bin/bash

# ==============================================
# ROS bag
# function：
# 1. get user name
# 2. record in bag file
# 3. record ROS topics
# ==============================================

# --------------------------
# 1. ROS init
# --------------------------
source /opt/ros/noetic/setup.bash

# --------------------------
# 2. get user name
# --------------------------
USER=${USER:-$(whoami)}
echo "user name: $USER"

# --------------------------
# 3. create bag file
# --------------------------
DATE_FOLDER=$(date +"%y%m%d")
BAG_DIR="/home/${USER}/bagfiles/${DATE_FOLDER}"
# create if not exist
mkdir -p "$BAG_DIR" 

# --------------------------
# 4. define Topics
# --------------------------
# ctrl
TOPICS_CTRL="/ctrl_pose/filtered /ctrl_pose/odom /ctrl_pose/pose  \
/mavros/vision_pose/pose /mavros/vision_pose/odom \
/mavros/local_position/pose /mavros/state \
/mavros/setpoint_position/local /mavros/setpoint_raw/attitude \
/controller/drone_target_euler_thrust /controller/drone_target_pose \
/controller/pid  /controller/drone_rc_feed /mavros/rc/in"

# UAV sensor
TOPICS_UAVSOR="/uav201/mocap/pos /uav201/mocap/vel \
/uwba0/mocap/pos /uwba0/mocap/vel \
/nlink_linktrack_nodeframe3 \
/mavros/imu/data /IMU_data \
/euler/IMU_data /euler/vicon /euler/mavros"

# CAM
TOPICS_PNPSOR="/camA/infra1/image_rect_raw/compressed /camA/infra2/image_rect_raw/compressed \
/camA/single_cam_process_ros/ir_mono/T_base_to_estimation \
/T_servogroup12_to_camA \
/T_base_to_servogroup12 \
/servo1_angle /servo2_angle"

# rosbag
TOPICS_ROSBAG="/rosbag/xt_f/pose /rosbag/xt_f/vel\
/rosbag/xt/pose /rosbag/xt/vel \
/rosbag/viconxt/pose /rosbag/viconxt/vel \
/rosbag/xt_real/pose /rosbag/xt_real/vel \
/rosbag/viconxt_real/pose /rosbag/viconxt_real/vel \
/rosbag/xt_ugv_f/pose /rosbag/viconxt_ugv_f/pose \
/rosbag/used/imu /rosbag/used/dt \
/rosbag/used/uwb /rosbag/used/uwbvicon \
/rosbag/used/flow \
/rosbag/used/hgt \
/rosbag/used/relative_flow \
/rosbag/used/cameraA \
/rosbag/param/uwb /rosbag/param/cam \
/rosbag/param/flow /rosbag/param/imu \
/rosbag/param/flow /rosbag/param/imu \
/rosbag/cov /rosbag/param/quality /rosbag/param/fail"

# onboard
TOPICS_ONBOARD="/estimate/xt_f/pose /estimate/xt_f/vel\
/estimate/xt/pose /estimate/xt/vel \
/estimate/viconxt/pose /estimate/viconxt/vel \
/estimate/xt_real/pose /estimate/xt_real/vel \
/estimate/viconxt_real/pose /estimate/viconxt_real/vel \
/estimate/xt_ugv_f/pose /estimate/viconxt_ugv_f/pose \
/estimate/used/imu /estimate/used/dt \
/estimate/used/uwb /estimate/used/uwbvicon \
/estimate/used/flow \
/estimate/used/hgt \
/estimate/used/relative_flow \
/estimate/used/cameraA \
/estimate/param/uwb /estimate/param/cam \
/estimate/param/flow /estimate/param/imu \
/estimate/param/flow /estimate/param/imu \
/estimate/cov /estimate/param/quality /estimate/param/fail"

# topics
TOPICS="$TOPICS_CTRL $TOPICS_UAVSOR $TOPICS_PNPSOR $TOPICS_ROSBAG $TOPICS_ONBOARD"

# --------------------------
# 5. start
# --------------------------
# enter the file
cd "$BAG_DIR"
# Set the output bag file name with timestamp 
OUTPUT_BAG_FILE="${BAG_DIR}/$(date +"%Y-%m-%d-%H-%M-%S").bag"
# record -o 
rosbag record $TOPICS
