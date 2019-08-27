# Human-Following Robot Using Human Skeleton and Pedestrian Dead Reckoning

- environment:

     NiTE2.2 + openNI2 + ubuntu 14.04 + ROS Indigo


- Equipped：

     Kinect v2 + DrRobot x80sv + samsung NOTE3


## following robot

     Quick demo
     $ roslaunch drrobot_X80_player following.launch

### kinect skeleton tracking
https://github.com/wsustcid/openni2_tracker
https://github.com/mcgi5sr2/kinect2_tracker

     1. to path file
     $ cd catkin_ws/src/NiTE-Linux-x64-2.2/Redist/
     
     2. Run tracking node
     $ rosrun kinect2_tracker human_tracker_show 


### robot
https://github.com/search?q=x80sv

     Run robot node
     $ roslaunch drrobot_X80_player robot.launch
     
### IMU
app: sensorstream IMU+GPS

https://play.google.com/store/apps/details?id=de.lorenz_fenster.sensorstreamgps


     1. smartphone, pc connect same Wi-Fi
     
     2. check ip addr
     $ ifconfig
     
     3. setup app ip and port
     ip : 192.168.88.106
     port: 5555
     
     4. Run imu node
     $ roslaunch imu_complementary_filter complementary_filter.launch  

### PDR

     1. open matlab

     $ cd Desktop/PDR

     $ matlab -nodesktop
     
     or

     $ cd usr/local/MATLAB/R2018b/bin/ && ./matlab

     2. what do u want
     
     - online calibrate step length parameter
     $ calibrate_PDR
     
     - PDR path following
     $ PDR_continuity

### joystick connect and setup
http://wiki.ros.org/ps3joy/Tutorials/PairingJoystickAndBluetoothDongle
http://wiki.ros.org/ps3joy

     connect USB port and joystick 

     $ roscd ps3joy

     $ sudo /usr/sbin/sixpair

     $ sudo hciconfig hci0 reset



### veloview

     - velodyne lidar
     $ cd VeloView-3.5.0-Linux-64bit/bin && ./VeloView




### sick
https://github.com/uos/sick_tim

     1. check usb port
     $ lsusb
     

     2. access permissions
     $ sudo chmod 777 /dev/bus/usb/001/001
     
     
     3.Run sick node
     $ roslaunch sick_tim sick_tim551_2050001.launch

## ref
- [1]  K. Koide and J. Miura, “Identification of a specific person using color, height, and gait features for a person following robot,” Robotics and Autonomous Systems, vol. 84, pp. 76–87, Oct. 2016. 
- [2]  M. Ota, T. Ogitsu, H. Hisahara, H. Takemura, Y. Ishii, and H. Mizoguchi, “Recovery function for human following robot losing target,” IECON 2013 - 39th Annual Conference of the IEEE Industrial Electronics Society, Nov. 2013.
- [3]  P. Nikdel, R. Shrestha, and R. Vaughan, “The Hands-Free Push-Cart: Autonomous Following in Front by Predicting User Trajectory Around Obstacles,” 2018 IEEE International Conference on Robotics and Automation (ICRA), May 2018.
- [4]  M. Gupta, S. Kumar, L. Behera, and V. K. Subramanian, “A Novel Vision-Based Tracking Algorithm for a Human-Following Mobile Robot,” IEEE Transactions on Systems, Man, and Cybernetics: Systems, vol. 47, no. 7, pp. 1415–1427, Jul. 2017.
- [5]  C. A. Cifuentes, A. Frizera, R. Carelli, and T. Bastos, “Human–robot interaction based on wearable IMU sensor and laser range finder,” Robotics and Autonomous Systems, vol. 62, no. 10, pp. 1425–1439, Oct. 2014.
- [6]  E. Babaians, N. Khazaee Korghond, A. Ahmadi, M. Karimi, and S. S. Ghidary, “Skeleton and visual tracking fusion for human following task of service robots,” 2015 3rd RSI International Conference on Robotics and Mechatronics (ICROM), Oct. 2015.
- [7]  M. Cao and H. Hashimoto, “Specific person recognition and tracking of mobile robot with Kinect 3D sensor,” IECON 2013 39th Annual Conference of the IEEE Industrial Electronics Society, Nov. 2013.
- [8]  R. Tasaki, H. Sakurai, and K. Terashima, “Moving target localization method using foot mounted acceleration sensor for autonomous following robot,” 2017 IEEE Conference on Control Technology and Applications (CCTA), Aug. 2017.
- [9]  B. X. Chen, R. Sahdev, and J. K. Tsotsos, “Integrating Stereo Vision with a CNN Tracker for a Person-Following Robot,” Lecture Notes in Computer Science, pp. 300–313, 2017.
- [10]  M. D. Hoang, S.-S. Yun, and J.-S. Choi, “The reliable recovery mechanism for person-following robot in case of missing target,” 2017 14th International Conference on Ubiquitous Robots and Ambient Intelligence (URAI), Jun. 2017.
- [11]  E. M. Diaz, A. L. M. Gonzalez, and F. de Ponte Muller, “Standalone inertial pocket navigation system,” 2014 IEEE/ION Position, Location and Navigation Symposium - PLANS 2014, May 2014
- [12]  H. Weinberg, “Using the ADXL202 in Pedometer and Personal Navigation Applications,” 2002.
- [13]  N.-H. Ho, P. Truong, and G.-M. Jeong, “Step-Detection and Adaptive Step-Length Estimation for Pedestrian Dead-Reckoning at Various Walking Speeds Using a Smartphone,” Sensors, vol. 16, no. 9, p. 1423, Sep. 2016.
- [14]  A. R. Jiménez, F. Seco, F. Zampella, J. C. Prieto, and J. Guevara, “PDR with a Foot-Mounted IMU and Ramp Detection,” Sensors, vol. 11, no. 10, pp. 9393–9410, Sep. 2011.
- [15]  S. K. Park and Y. S. Suh, “A Zero Velocity Detection Algorithm Using Inertial Sensors for Pedestrian Navigation Systems,” Sensors, vol. 10, no. 10, pp. 9163–9178, Oct. 2010.
- [16]  E. M. Diaz and A. L. M. Gonzalez, “Step detector and step length estimator for an inertial pocket navigation system,” 2014 International Conference on Indoor Positioning and Indoor Navigation (IPIN), Oct. 2014.
- [17]  E. Munoz Diaz, “Inertial Pocket Navigation System: Unaided 3D Positioning,” Sensors, vol. 15, no. 4, pp. 9156–9178, Apr. 2015.
- [18]  V. Renaudin, M. Susi, and G. Lachapelle, “Step Length Estimation Using Handheld Inertial Sensors,” Sensors, vol. 12, no. 7, pp. 8507–8525, Jun. 2012.
- [19]  J. Qian, L. Pei, J. Ma, R. Ying, and P. Liu, “Vector Graph Assisted Pedestrian Dead Reckoning Using an Unconstrained Smartphone,” Sensors, vol. 15, no. 3, pp. 5032–5057, Mar. 2015.
- [20]  Jiuchao Qian, Jiabin Ma, Rendong Ying, Peilin Liu, and Ling Pei, “An improved indoor localization method using smartphone inertial sensors,” International Conference on Indoor Positioning and Indoor Navigation, Oct. 2013.
- [21]  M. P. R. S. Kiran, P. Rajalakshmi, M. K. Giluka, and B. R. Tamma, “A novel system architecture for real-time, robust and accurate step detection for PDR based indoor localization,” 2018 IEEE 4th World Forum on Internet of Things (WF-IoT), Feb. 2018.
- [22]  H. Lee, S. Choi, and M. Lee, “Step Detection Robust against the Dynamics of Smartphones,” Sensors, vol. 15, no. 10, pp. 27230–27250, Oct. 2015.
- [23]  L. E. Diez, A. Bahillo, J. Otegui, and T. Otim, “Step Length Estimation Methods Based on Inertial Sensors: A Review,” IEEE Sensors Journal, vol. 18, no. 17, pp. 6908–6926, Sep. 2018.
- [24]  C. Tunca, N. Pehlivan, N. Ak, B. Arnrich, G. Salur, and C. Ersoy, “Inertial Sensor-Based Robust Gait Analysis in Non-Hospital Settings for Neurological Disorders,” Sensors, vol. 17, no. 4, p. 825, Apr. 2017.
- [25]  B. Shin, C. Kim, J. Kim, S. Lee, C. Kee, and T. Lee, “Motion Recognition based 3D Pedestrian Navigation System using Smartphone,” IEEE Sensors Journal, pp. 1–1, 2016.
- [26]  O. D. Incel, “Analysis of Movement, Orientation and Rotation-Based Sensing for Phone Placement Recognition,” Sensors (Basel, Switzerland), vol. 15, no. 10, pp. 25474–506, 2015.
- [27]  Y. Cui, J. Chipchase, and F. Ichikawa, “A Cross Culture Study on Phone Carrying and Physical Personalization,” Usability and Internationalization. HCI and Culture, pp. 483–492, 2007.
- [28]  A. Ohya, T. Munekata,“ Intelligent Escort Robot Moving together with Human-Interaction in Accompanying Behavior, “ in: Proceedings of the 2002 FIRA Robot Congress, pp. 31–35 , 2002
- [29]  S. S. Honig, T. Oron-Gilad, H. Zaichyk, V. Sarne-Fleischmann, S. Olatunji and Y. Edan, "Toward Socially Aware Person-Following Robots," in IEEE Transactions on Cognitive and Developmental Systems, vol. 10, no. 4, pp. 936-954, Dec. 2018.
- [30]  R. Gockley, J. Forlizzi and R. Simmons, "Natural person-following behavior for social robots," 2007 2nd ACM/IEEE International Conference on Human-Robot Interaction (HRI), Arlington, VA, 2007, pp. 17-24.
- [31]  R. Valenti, I. Dryanovski, and J. Xiao, “Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs,” Sensors, vol. 15, no. 8, pp. 19302–19330, Aug. 2015.
- [32]  P. Davidson and R. Piché, "A Survey of Selected Indoor Positioning Methods for Smartphones," in IEEE Communications Surveys & Tutorials, vol. 19, no. 2, pp. 1347-1370, Secondquarter 2017.
- [33]  L. E. Díez, A. Bahillo, J. Otegui and T. Otim, "Step Length Estimation Methods Based on Inertial Sensors: A Review," in IEEE Sensors Journal, vol. 18, no. 17, pp. 6908-6926, 1 Sept.1, 2018.
- [34]  R. Harle, "A Survey of Indoor Inertial Positioning Systems for Pedestrians," in IEEE Communications Surveys & Tutorials, vol. 15, no. 3, pp. 1281-1293, Third Quarter 2013.
