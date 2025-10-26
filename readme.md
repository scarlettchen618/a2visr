# a2VISR

## Brief introduction
a2VISR for state estimation

## Download
```
git clone https://gitee.com/scarlettchen/a2visr_est.git
cd ..
catkin_make
```
## Topics
- estimation: /estimate/xt_f/pose /estimate/xt_f/vel
- rosbag test: /rosbag/xt_f/pose /rosbag/xt_f/vel
- Mocap: /uav201/mocap/pos /uav201/mocap/vel /uwba0/mocap/pos 

## Launch
- rosbag check
```
roslaunch a2VISR a2VISR_multi_rosbag.launch
```
- online estimation
```
roslaunch a2VISR a2VISR_multi_onboard.launch
```
- record bags
```
roscd a2VISR/sh
bash est_record.sh
```
