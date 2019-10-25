# Human-Following Robot Using Human Skeleton and Pedestrian Dead Reckoning

- environment:

     NiTE2.2 + openNI2 + ubuntu 14.04 + ROS Indigo


- Equipped：

     Kinect v2 + DrRobot x80sv + samsung NOTE3
     

## following robot
- Quick demo(kinect + imu + robot)
```bash
     $ roslaunch drrobot_X80_player following.launch 
```     

![Alt text](https://github.com/Yu1107/following-robot/blob/master/system.png)

### kinect skeleton tracking
https://github.com/wsustcid/openni2_tracker

https://github.com/mcgi5sr2/kinect2_tracker

1. to path file
```bash
     $ cd catkin_ws/src/NiTE-Linux-x64-2.2/Redist/
```     
2. Run tracking node
```bash
     $ rosrun kinect2_tracker human_tracker_show 
```

### robot
https://github.com/search?q=x80sv

- Run robot node
```bash
     $ roslaunch drrobot_X80_player robot.launch
```
     
### IMU
app: sensorstream IMU+GPS

https://play.google.com/store/apps/details?id=de.lorenz_fenster.sensorstreamgps


1. smartphone, pc connect same Wi-Fi
     
2. check ip addr
     ```bash
     $ ifconfig
    ``` 
3. setup app ip and port

     ip : 192.168.88.106
     
     port: 5555
     
4. Run imu node
     ```bash
     $ roslaunch imu_complementary_filter complementary_filter.launch  
     ```

### PDR

1. open matlab

```bash
     $ cd Desktop/PDR

     $ matlab -nodesktop
     
     or

     $ cd usr/local/MATLAB/R2018b/bin/ && ./matlab
```
2. what do u want(mission)
```bash     
     - online calibrate step length parameter
     $ calibrate_PDR
     
     - PDR path following
     $ PDR_continuity
```
### joystick connect and setup
http://wiki.ros.org/ps3joy/Tutorials/PairingJoystickAndBluetoothDongle

http://wiki.ros.org/ps3joy


- connect USB port and joystick 

```bash
     $ roscd ps3joy

     $ sudo /usr/sbin/sixpair

     $ sudo hciconfig hci0 reset
```


### veloview lidar
     
     $ cd VeloView-3.5.0-Linux-64bit/bin && ./VeloView




### sick
https://github.com/uos/sick_tim


1. check usb port
```bash
    $ lsusb
 ```    

2. access permissions
```bash
     $ sudo chmod 777 /dev/bus/usb/001/001
 ```    
     
3.Run sick node
```bash     
     $ roslaunch sick_tim sick_tim551_2050001.launch
```
