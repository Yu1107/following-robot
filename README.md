# following-robot

- environment:

NiTE2.2 + openNI2 + ubuntu 14.04

- Equipped：
Kinect v2 + DrRobot x80sv + samsung NOTE2 + WIRELESS ROUTER

# following robot

$ roslaunch drrobot_X80_player following.launch


# PDR

1.open matlab

$ cd Desktop/PDR
$ matlab -nodesktop
or
$ cd usr/local/MATLAB/R2018b/bin/ && ./matlab

2.choose 
$ calibrate_PDR

$ PDR_continuity


# joystick connect and setup
- connect USB port and joystick

$ roscd ps3joy

$ sudo /usr/sbin/sixpair

$ sudo hciconfig hci0 reset



# veloview

- velodyne lidar


$ cd VeloView-3.5.0-Linux-64bit/bin && ./VeloView




# sick

1. check usb port


$ lsusb

2. access permissions


$ sudo chmod 777 /dev/bus/usb/001/001
