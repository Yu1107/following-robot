# following-robot


------following robot------
roslaunch drrobot_X80_player following.launch

cd Desktop/
rosbag record -a


------matlab------
1.
cd Desktop/PDR
matlab -nodesktop

calibrate_PDR

PDR_continuity

2.
cd usr/local/MATLAB/R2018b/bin/ && ./matlab

------joystick commect and setup------
roscd ps3joy
sudo /usr/sbin/sixpair

sudo hciconfig hci0 reset


------veloview------
cd VeloView-3.5.0-Linux-64bit/bin && ./VeloView

------sick------
lsusb
sudo chmod 777 
