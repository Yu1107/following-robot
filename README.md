# following-robot

environment:

NiTE2.2 + openNI2 + ubuntu 14.04

Equipped：

Kinect v2 + DrRobot x80sv + samsung NOTE2 + WIRELESS ROUTER

------following robot------

$ roslaunch drrobot_X80_player following.launch


------PDR------
open matlab

1.

$ cd Desktop/PDR
$ matlab -nodesktop

$ calibrate_PDR

$ PDR_continuity

2.
$ cd usr/local/MATLAB/R2018b/bin/ && ./matlab





------joystick connect and setup------

$ roscd ps3joy
$ sudo /usr/sbin/sixpair

$ sudo hciconfig hci0 reset



------veloview------

velodyne lidar


$ cd VeloView-3.5.0-Linux-64bit/bin && ./VeloView




------sick------

1. check usb port
$ lsusb

2. access permissions
$ sudo chmod 777 /dev/bus/usb/001/001
