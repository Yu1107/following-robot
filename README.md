# Combining human skeleton and pedestrian dead reckoning for research of following robot

- environment:

     NiTE2.2 + openNI2 + ubuntu 14.04 + ROS Indigo


- Equipped：

     Kinect v2 + DrRobot x80sv + samsung NOTE3


## following robot

     Quick demo
     $ roslaunch drrobot_X80_player following.launch

### kinect skeleton tracking

     1. to path file
     $ cd catkin_ws/src/NiTE-Linux-x64-2.2/Redist/
     
     2. Run tracking node
     $ rosrun kinect2_tracker human_tracker_show 


### robot
     Run robot node
     $ roslaunch drrobot_X80_player robot.launch
     
### IMU
- app(need to download):Sensorstream IMU+GPS:
https://play.google.com/store/apps/details?id=de.lorenz_fenster.sensorstreamgps

     1.smartphone, pc connect same Wi-Fi
     
     2. check ip addr
     $ ifconfig
     
     3. setup app ip and port
     ip : 192.168.88.106
     port: 5555
     
     4. Run imu node
     $ roslaunch imu_complementary_filter complementary_filter.launch  

### PDR

     1.open matlab

     $ cd Desktop/PDR

     $ matlab -nodesktop
     
     or

     $ cd usr/local/MATLAB/R2018b/bin/ && ./matlab

     2.choose task

     $ calibrate_PDR

     $ PDR_continuity

### joystick connect and setup
     - connect USB port and joystick

     $ roscd ps3joy

     $ sudo /usr/sbin/sixpair

     $ sudo hciconfig hci0 reset



### veloview

     - velodyne lidar
     $ cd VeloView-3.5.0-Linux-64bit/bin && ./VeloView





### sick

     1. check usb port
     $ lsusb
     

     2. access permissions
     $ sudo chmod 777 /dev/bus/usb/001/001
     
     
     3.Run sick node
     $ roslaunch sick_tim sick_tim551_2050001.launch
