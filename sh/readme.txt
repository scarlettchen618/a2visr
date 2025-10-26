1. test with vicon

roslaunch vrpn_client_ros sample.launch //

roslaunch nlink_parser linktrack.launch //

roslaunch mavros px4.launch // 

roslaunch serial_imu trans_imu_node.launch //

rostopic echo /mavros/vision_pose/pose //

rostopic echo /csj01/mocap/pos //

roslaunch offboard attctl_circle.launch
==

2. test with vicon and est

roslaunch vrpn_client_ros sample.launch //

roslaunch nlink_parser linktrack.launch //

roslaunch mavros px4.launch //

roslaunch serial_imu trans_imu_node.launch //

roslaunch a2VISR a2VISR_multi_onboard.launch //      

rostopic echo /mavros/vision_pose/pose //
 
rostopic echo /csj01/mocap/pos //

roslaunch offboard attctl_circle.launch
