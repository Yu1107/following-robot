/*!
 *  drrobo_X80_player
 *  Copyright (c) 2011, Dr Robot Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*!

@mainpage
  drrobot_X80_player is a driver for motion control system on X80 series mobile robot, available from
<a href="http://www.drrobot.com">Dr Robot </a>.
<hr>

@verbatim
$ drrobot_X80_player
@endverbatim

<hr>
@section topic ROS topics

Subscribes to (name/type):
- @b "cmd_vel"/Twist : velocity commands to differentially drive the robot.
- @b will develop other command subscribles in future, such as servo control.

Publishes to (name / type):
-@b drrobot_motor: will publish MotionInfoArray Message. Please referee the message file.
-@b drrobot_powerinfo: will publish PowerInfo Message. Please referee the message file.
-@b drrobot_ir: will publish RangeArray Message for IR sensor, and transform AD value from DrRobotMotionSensorDriver to distance value in unit meter. Please referee the message file.
-@b drrobot_sonar: will publish RangeArray Message for ultrasonic sensor, and transform value from DrRobotMotionSensorDriver to distance value in unit meter. Please referee the message file.
-@b drrobot_standardsensor: will publish StandardardSensor Message. Please referee the message file.
-@b drrobot_customsensor: will publish CustomSensor Message. Please referee the message file. Not available for standard I90/Sentinel3/Hawk/H20/X80SV robot

<hr>

@section parameters ROS parameters, please read yaml file

- @b RobotCommMethod (string) : Robot communication method, normally is "Network".
- @b RobotID (string) : specify the robot ID
- @b RobotBaseIP (string) : robot main WiFi module IP address in dot format, default is "192.168.0.201".
- @b RobotPortNum (string) : socket port number first serial port, and as default the value increased by one will be second port number.
- @b RobotSerialPort (int) : specify the serial port name if you choose serial communication in RobotCommMethod, default /dev/ttyS0"
- @b RobotType (string) : specify the robot type, now should in list: I90, Sentinel3, Hawk_H20, Jaguar, X80SV
- @b MotorDir (int) : specify the motor control direction
- @b WheelRadius (double) : wheel radius
- @b WheelDistance (double) : the distance between two driving wheels
- @b EncoderCircleCnt (int) : one circle encoder count
- @b MinSpeed (double) : minimum speed, unit is m/s.
- @b MaxSpeed (double) : maximum speed, unit is m/s.
- @b enable_ir (bool)  : Whether to enable sonar range sensors. Default: true.
- @b enable_sonar (bool)  : Whether to enable IR range sensors. Default: true.
 */

#include <assert.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>

#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Range.h>

#include <drrobot_X80_player/MotorInfo.h>
#include <drrobot_X80_player/MotorInfoArray.h>
#include <drrobot_X80_player/RangeArray.h>
#include <drrobot_X80_player/Range.h>
#include <drrobot_X80_player/PowerInfo.h>
#include <drrobot_X80_player/StandardSensor.h>
#include <drrobot_X80_player/CustomSensor.h>
#include <DrRobotMotionSensorDriver.hpp>

#define MOTOR_NUM 3
#define IR_NUM 7
#define US_NUM 3
using namespace std;
using namespace DrRobot_MotionSensorDriver;
using namespace boost::assign;

class DrRobotPlayerNode
{
public:
  ros::NodeHandle node_;

  tf::TransformBroadcaster m_odom_broadcaster;

  ros::Publisher motorInfo_pub_;
  ros::Publisher powerInfo_pub_;
  ros::Publisher ir_pub_;
  ros::Publisher sonar_pub_;
  ros::Publisher standardSensor_pub_;
  ros::Publisher customSensor_pub_;

  ros::Publisher m_odom_pub;
  ros::Publisher m_joint_state;
  ros::Publisher path_pub;
  ros::Publisher range_cloud_pub_;

  ros::Subscriber cmd_vel_sub_;
  std::string robot_prefix_;

  DrRobotPlayerNode()
  {
    ros::NodeHandle private_nh("~");

    robotID_ = "drobot1";
    private_nh.getParam("RobotID", robotID_);
    ROS_INFO("I get ROBOT_ID: [%s]", robotID_.c_str());

    robotType_ = "X80"; //now fixed as X80 robot
    private_nh.getParam("RobotType", robotType_);
    ROS_INFO("I get ROBOT_Type: [%s]", robotType_.c_str());

    robotCommMethod_ = "Network";
    private_nh.getParam("RobotCommMethod", robotCommMethod_);
    ROS_INFO("I get ROBOT_CommMethod: [%s]", robotCommMethod_.c_str());

    robotIP_ = "192.168.0.201"; // you could modify the IP as your robot IP
    private_nh.getParam("RobotBaseIP", robotIP_);
    ROS_INFO("I get ROBOT_IP: [%s]", robotIP_.c_str());

    commPortNum_ = 10001;
    private_nh.getParam("RobotPortNum", commPortNum_);
    ROS_INFO("I get ROBOT_PortNum: [%d]", commPortNum_);

    robotSerialPort_ = "/dev/ttyS0";
    private_nh.getParam("RobotSerialPort", robotSerialPort_);
    ROS_INFO("I get ROBOT_SerialPort: [%s]", robotSerialPort_.c_str());

    enable_ir_ = true;

    private_nh.getParam("Enable_IR", enable_ir_);
    if (enable_ir_)
      ROS_INFO("I get Enable_IR: true");
    else
      ROS_INFO("I get Enable_IR: false");

    enable_sonar_ = true;

    private_nh.getParam("Enable_US", enable_sonar_);
    if (enable_sonar_)
      ROS_INFO("I get Enable_US: true");
    else
      ROS_INFO("I get Enable_US: false");

    motorDir_ = -1;
    private_nh.getParam("MotorDir", motorDir_);
    ROS_INFO("I get MotorDir: [%d]", motorDir_);

    wheelRadius_ = 0.09;
    private_nh.getParam("WheelRadius", wheelRadius_);
    ROS_INFO("I get Wheel Radius: [%f]", wheelRadius_);

    wheelDis_ = 0.283;
    private_nh.getParam("WheelDistance", wheelDis_);
    ROS_INFO("I get Wheel Distance: [%f]", wheelDis_);

    minSpeed_ = 0.0;
    private_nh.getParam("MinSpeed", minSpeed_);
    ROS_INFO("I get Min Speed: [%f]", minSpeed_);

    maxSpeed_ = 2.0;
    private_nh.getParam("MaxSpeed", maxSpeed_);
    ROS_INFO("I get Max Speed: [%f]", maxSpeed_);

    encoderOneCircleCnt_ = 756; // you need modify these values with your robot hardware
    private_nh.getParam("EncoderCircleCnt", encoderOneCircleCnt_);
    ROS_INFO("I get Encoder One Circle Count: [%d]", encoderOneCircleCnt_);

    if (robotCommMethod_ == "Network")
    {
      robotConfig1_.commMethod = Network;
      robotConfig2_.commMethod = Network;
    }
    else
    {
      robotConfig1_.commMethod = Serial;
      robotConfig2_.commMethod = Serial;
    }

    robotConfig1_.boardType = X80;

    robotConfig1_.portNum = commPortNum_;

    //  strcat(robotConfig1_.robotIP,robotIP_.c_str());
    strcpy(robotConfig1_.robotIP, robotIP_.c_str());

    //  strcat(robotConfig1_.serialPortName,robotSerialPort_.c_str());
    strcpy(robotConfig1_.serialPortName, robotSerialPort_.c_str());

    //create publishers for sensor data information
    motorInfo_pub_ = node_.advertise<drrobot_X80_player::MotorInfoArray>("drrobot_motor", 1);

    if (enable_ir_)
    {
      ir_pub_ = node_.advertise<drrobot_X80_player::RangeArray>("drrobot_ir", 1);
    }
    if (enable_sonar_)
    {
      sonar_pub_ = node_.advertise<drrobot_X80_player::RangeArray>("drrobot_sonar", 1);
    }
    //standardSensor_pub_ = node_.advertise<drrobot_X80_player::StandardSensor>("drrobot_standardsensor", 1);
    //customSensor_pub_ = node_.advertise<drrobot_X80_player::CustomSensor>("drrobot_customsensor", 1);

    range_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud>("range_cloud", 3);
    m_odom_pub = node_.advertise<nav_msgs::Odometry>("odom", 1);
    m_joint_state = node_.advertise<sensor_msgs::JointState>("joint_states", 1);
    path_pub = node_.advertise<nav_msgs::Path>("trajectory", 10, true);

    drrobotMotionDriver_ = new DrRobotMotionSensorDriver();
    drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);

    cntNum_ = 0;

    // Keep track of robot:
    m_x = 0;
    m_y = 0;
    m_theta = 0;
  }

  ~DrRobotPlayerNode()
  {
  }

  int start()
  {

    int res = -1;

    drrobotMotionDriver_->openNetwork(robotConfig1_.robotIP, robotConfig1_.portNum);
    cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&DrRobotPlayerNode::cmdVelReceived, this, _1));

    // only needed when using velocity (1, 0, 170))
    drrobotMotionDriver_->setMotorVelocityCtrlPID(0, 15, 0, 0); // channel, p, d, i
    drrobotMotionDriver_->setMotorVelocityCtrlPID(1, 15, 0, 0);

    // PID default is 1000, 5, 10000 (taken from C# src)
    //drrobotMotionDriver_->setMotorPositionCtrlPID(0, 500, 5, 10000);
    //drrobotMotionDriver_->setMotorPositionCtrlPID(1, 500, 5, 10000);
    drrobotMotionDriver_->setMotorPositionCtrlPID(0, 1000, 5, 10000);
    drrobotMotionDriver_->setMotorPositionCtrlPID(1, 1000, 5, 10000);
    // 500, 2, 510 has smoother motion but does not take friction in account

    return (0);
  }

  int stop()
  {
    int status = 0;
    drrobotMotionDriver_->close();
    drrobotPowerDriver_->close();
    usleep(1000000);
    return (status);
  }

  void cmdVelReceived(const geometry_msgs::Twist::ConstPtr &cmd_vel)
  {
    double g_vel = cmd_vel->linear.x;
    double t_vel = cmd_vel->angular.z;
    if (robotConfig1_.boardType != Jaguar)
    {
      double leftWheel = (2 * g_vel - t_vel * wheelDis_) / (2 * wheelRadius_);
      double rightWheel = (t_vel * wheelDis_ + 2 * g_vel) / (2 * wheelRadius_);

      int leftWheelCmd = motorDir_ * leftWheel * encoderOneCircleCnt_ / (2 * 3.1415927);
      int rightWheelCmd = -motorDir_ * rightWheel * encoderOneCircleCnt_ / (2 * 3.1415927);
      ROS_INFO("Received control command: [%d, %d]", leftWheelCmd, rightWheelCmd);
      drrobotMotionDriver_->sendMotorCtrlAllCmd(Velocity, leftWheelCmd, rightWheelCmd, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL);
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //add function

  void calculateMovementDelta(drrobot_X80_player::MotorInfo &mtr, int &encoderPrevious, double &movementDelta, double &joint_angle)
  {
    const uint encoderMax = 32768;

    if (encoderPrevious != -1)
    {
      int encoderDelta = 0;

      if (encoderPrevious < 1000 && mtr.encoder_pos > 31000)
      {
        // When the encoder wrap around from lower to upper value
        // moving backwards. Encoderdelta must be negative number:
        encoderDelta = -((encoderMax - mtr.encoder_pos) + encoderPrevious);
      }
      else if (encoderPrevious > 31000 && mtr.encoder_pos < 1000)
      {
        // When moving from top to bottom, we move forward, so
        // delta must be positive:
        encoderDelta = (encoderMax - encoderPrevious) + mtr.encoder_pos;
      }
      else
      {
        // The 'normal' case:
        encoderDelta = mtr.encoder_pos - encoderPrevious;
      }

      movementDelta = encoder2rad(encoderDelta);
    }
    else
    {
      movementDelta = 0;
    }

    joint_angle = encoder2rad(mtr.encoder_pos);
    encoderPrevious = mtr.encoder_pos;
  }

  /*
        This function calculates odometry information from the encoderdata.
        It creates a transform from 'odom' to 'base_footprint'

        motor 0 is the left wheel, motor 1 the right wheel.

        The x axis is forward, the y axis is to the left, and the z is upwards.

        The base footprint is exactly between the wheels. The base link is the center of the robot.
    */
  void publishOdometry(const drrobot_X80_player::MotorInfoArray &motorInfo)
  {
    static int mEncoderPreviousLeft = -1; //TODO confirm left and right are correct
    static int mEncoderPreviousRight = -1;
    static ros::Time prev_time(0);

    ros::Time nu = ros::Time::now();

    double time_delta = (nu - prev_time).toSec();
    prev_time = nu;
    if (time_delta < 0.0001)
    {
      time_delta = 0.0001;
    }

    double left_joint_angle;
    double right_joint_angle;

    drrobot_X80_player::MotorInfo mtr0 = motorInfo.motorInfos.at(0); // Left motor
    drrobot_X80_player::MotorInfo mtr1 = motorInfo.motorInfos.at(1); // Right motor

    // ROS_INFO("Encoder values: %u, %u", mtr0.encoder_pos, mtr1.encoder_pos);

    double left_movement_delta;
    double right_movement_delta;

    calculateMovementDelta(mtr0, mEncoderPreviousLeft, left_movement_delta, left_joint_angle);
    calculateMovementDelta(mtr1, mEncoderPreviousRight, right_movement_delta, right_joint_angle);

    // TODO: take into account the wheel direction:
    double d_left = -left_movement_delta * wheelRadius_;
    double d_right = right_movement_delta * wheelRadius_;

    // average distance between 2 wheels = actual distance from center
    double averageDistance = (d_left + d_right) / 2.0;

    // difference in angle from last encoder values, distance between wheel centers in m
    // When the right wheel moves forward, the angle
    double deltaAngle = atan2((d_right - d_left), wheelDis_);
    // ROS_INFO("Left movement %f, right %f, angle=%f", d_left, d_right, deltaAngle);
    double delta_x = averageDistance * cos(m_theta);
    double delta_y = averageDistance * sin(m_theta);

    // x is in forward direction
    // y is sideways.
    // Angle is zero when facing forwards.

    // TODO: retrieve velocities:
    double vx = averageDistance / time_delta;
    double vy = 0;
    double vth = deltaAngle / time_delta;

    // update pose:
    m_theta += deltaAngle;
    m_x += delta_x;
    m_y += delta_y;

    // ROS_INFO("Robot pos: %f, %f", m_x, m_y);

    // Send out joint state via topic:
    // The joint name is a combination of the erratic wheel and the xacro description.
    // This is published into static tf via the robot_state_publisher.
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = nu;
    joint_state.name.resize(4);
    joint_state.position.resize(4);
    joint_state.name[0] = "base_link_left_wheel_joint";
    joint_state.position[0] = left_joint_angle;
    joint_state.name[1] = "base_link_right_wheel_joint";
    joint_state.position[1] = right_joint_angle;

    // These are two joints we cannot measure:
    joint_state.name[2] = "base_caster_support_joint";
    joint_state.position[2] = 0;
    joint_state.name[3] = "caster_wheel_joint";
    joint_state.position[3] = 0;
    m_joint_state.publish(joint_state);

    // Grab some constant things:
    tf::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, m_theta);

    // Construct tf message:
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(m_x, m_y, 0.0));
    transform.setRotation(odom_quat);

    // Send via tf system:
    m_odom_broadcaster.sendTransform(tf::StampedTransform(transform, nu, "odom", "base_footprint"));

    // Construct odometry message:
    nav_msgs::Odometry odom;
    odom.header.stamp = nu;
    odom.header.frame_id = "odom";

    // Set position:
    odom.pose.pose.position.x = m_x;
    odom.pose.pose.position.y = m_y;

    geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(m_theta);
    odom.pose.pose.orientation = odom_quat2;

    // Set velocity:
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    // Publish the message:
    m_odom_pub.publish(odom);

    nav_msgs::Path path;
    path.header.stamp = nu;
    path.header.frame_id = "odom";
    geometry_msgs::PoseStamped this_pose_stamped;

    this_pose_stamped.pose.position.x = m_x;
    this_pose_stamped.pose.position.y = m_y;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(m_theta);
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp = nu;
    this_pose_stamped.header.frame_id = "odom";
    path.poses.push_back(this_pose_stamped);

    path_pub.publish(path);

    ros::spinOnce(); // check for incoming messages
  }

  // Convert range to coordinates and add it into point cloud
  // This can be used to take both ir and us range data into a single pointcloud.
  // This function is aware of positions of the sensors
  void addRangeToCloud(const sensor_msgs::Range &range, sensor_msgs::PointCloud &pointCloud)
  {
    // convert range arrays here to RangeDefinedArray (this means adding the angle and distance toward centroid of robot)
    float angleFromCenterDegrees = 0;
    float distanceFromCenterMM = 0;
    float yOffsetFromCenterMM = 0;

    if (range.radiation_type == sensor_msgs::Range::ULTRASOUND)
    {
      // get sensor id number from the header.frame_id string
      string frame_id = range.header.frame_id;

      uint8_t sensorID = frame_id.at(14) - 48; // drrobot_sonar_# where # is the number we want, the prefix is defined in the drrobot_player.cpp file

      switch (sensorID)
      {
        // add angle and distance from centroid in this switchcase

      case 0:                          // front left
        angleFromCenterDegrees = 45;   //in degrees
        distanceFromCenterMM = 194.08; // in mm
        yOffsetFromCenterMM = -2.36;
        break;
      case 1: // middle
        angleFromCenterDegrees = 0;
        distanceFromCenterMM = 191.50;
        yOffsetFromCenterMM = 0;
        break;
      case 2: // front right
        angleFromCenterDegrees = -45;
        distanceFromCenterMM = 194.08;
        yOffsetFromCenterMM = 2.36;
        break;
      default:
        ROS_ERROR("invalid SONAR sensor ID found: %u", sensorID);
        return;
      }
    }
    /*   else if (range.radiation_type == sensor_msgs::Range::INFRARED)
        {
            // get sensor id number from the header.frame_id string
            string frame_id = range.header.frame_id;

            uint8_t sensorID = frame_id.at(11) - 48; // drrobot_ir_# where # is the number we want, the prefix is defined in the drrobot_player.cpp file

            switch (sensorID)
            {
                // add angle and distance from centroid in this switchcase
                // angle based on front (so straight ahead = 0 degrees)

                case 0: // front left
                    angleFromCenterDegrees = 33.75; //in degrees
                    distanceFromCenterMM = 190.82; // in mm
                    yOffsetFromCenterMM = -6.57;
                    break;
                case 1: // front left mid
                    angleFromCenterDegrees = 11.25;
                    distanceFromCenterMM = 186.76;
                    yOffsetFromCenterMM = 4.34;
                    break;
                case 2: // front right mid
                    angleFromCenterDegrees = -11.25;
                    distanceFromCenterMM = 186.76;
                    yOffsetFromCenterMM = -4.34;
                    break;
                case 3: // front right
                    angleFromCenterDegrees = -33.75;
                    distanceFromCenterMM = 190.82;
                    yOffsetFromCenterMM = 6.57;
                    break;
                case 4: // right side
                    angleFromCenterDegrees = -90;
                    distanceFromCenterMM = 121;
                    yOffsetFromCenterMM = 0;
                    break;
                case 5: // back side
                    angleFromCenterDegrees = -180;
                    distanceFromCenterMM = 186;
                    yOffsetFromCenterMM = 0;
                    break;
                case 6: // left side
                    angleFromCenterDegrees = 90;
                    distanceFromCenterMM = 121;
                    yOffsetFromCenterMM = 0;
                    break;
                default:
                    ROS_ERROR("invalid IR sensor ID found: %u", sensorID);
                    return;
            }
        }*/
    else
    {
      ROS_ERROR("invalid or NYI sensor type found");
    }

    // skip if not within limits:
    if (range.range > range.min_range && range.range < range.max_range)
    {
      float angleFromCenter = angleFromCenterDegrees / 180 * M_PI; // save as rads
      float distanceFromCenter = distanceFromCenterMM / 1000;      // save as meters
      float yOffsetFromCenter = yOffsetFromCenterMM / 1000;        // as meters

      double alpha = angleFromCenter;                     // sensor angle + current pose angle
      double distance = distanceFromCenter + range.range; // sensor range + offset from center

      geometry_msgs::Point32 p;
      p.x = (cos(alpha) * distance);                     // add current pose x
      p.y = (sin(alpha) * distance) + yOffsetFromCenter; // y
      float d = sqrt(p.x * p.x + p.y * p.y);
      //ROS_INFO("I get [%s] ,pose[%5.2f %5.2f]",range.header.frame_id.c_str(),p.x, p.y);
      pointCloud.points.push_back(p);
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void doUpdate()
  {
    drrobot_X80_player::MotorInfoArray motorInfoArray;
    motorInfoArray.motorInfos.resize(MOTOR_NUM);
    if (drrobotMotionDriver_->portOpen())
    {
      drrobotMotionDriver_->readMotorSensorData(&motorSensorData_);
      drrobotMotionDriver_->readRangeSensorData(&rangeSensorData_);
      //drrobotMotionDriver_->readStandardSensorData(&standardSensorData_);
      //drrobotMotionDriver_->readCustomSensorData(&customSensorData_);
      // Translate from driver data to ROS data
      cntNum_++;
      motorInfoArray.motorInfos.resize(MOTOR_NUM);
      for (uint32_t i = 0; i < MOTOR_NUM; ++i)
      {
        motorInfoArray.motorInfos[i].header.stamp = ros::Time::now();
        motorInfoArray.motorInfos[i].header.frame_id = string("drrobot_motor_");
        motorInfoArray.motorInfos[i].header.frame_id += boost::lexical_cast<std::string>(i);
        motorInfoArray.motorInfos[i].robot_type = robotConfig1_.boardType;
        motorInfoArray.motorInfos[i].encoder_pos = motorSensorData_.motorSensorEncoderPos[i];
        motorInfoArray.motorInfos[i].encoder_vel = motorSensorData_.motorSensorEncoderVel[i];
        motorInfoArray.motorInfos[i].encoder_dir = motorSensorData_.motorSensorEncoderDir[i];
        if (robotConfig1_.boardType == Hawk_H20_Motion)
        {
          motorInfoArray.motorInfos[i].motor_current = (float)motorSensorData_.motorSensorCurrent[i] * 3 / 4096;
          ;
        }
        else if (robotConfig1_.boardType != Jaguar)
        {
          motorInfoArray.motorInfos[i].motor_current = (float)motorSensorData_.motorSensorCurrent[i] / 728;
        }
        else
        {
          motorInfoArray.motorInfos[i].motor_current = 0.0;
        }
        motorInfoArray.motorInfos[i].motor_pwm = motorSensorData_.motorSensorPWM[i];
      }

      //ROS_INFO("publish motor info array");
      motorInfo_pub_.publish(motorInfoArray);

      sensor_msgs::PointCloud pointCloud;

      if (enable_sonar_)
      {
        drrobot_X80_player::RangeArray rangerArray;
        rangerArray.ranges.resize(US_NUM);
        for (uint32_t i = 0; i < US_NUM; ++i)
        {

          rangerArray.ranges[i].header.stamp = ros::Time::now();
          rangerArray.ranges[i].header.frame_id = string("drrobot_sonar_");
          rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
          rangerArray.ranges[i].range = (float)rangeSensorData_.usRangeSensor[i] / 100; //to meters

          // around 30 degrees
          rangerArray.ranges[i].field_of_view = 0.5236085;
          rangerArray.ranges[i].max_range = 2.55;
          rangerArray.ranges[i].min_range = 0.0;
          //rangerArray.ranges[i].radiation_type = drrobot_X80_player::Range::ULTRASOUND;

          rangerArray.ranges[i].radiation_type = sensor_msgs::Range::ULTRASOUND;
          addRangeToCloud(rangerArray.ranges[i], pointCloud);
          //ROS_INFO("I get [%s] ROBOT_ID:[%5.2f]",rangerArray.ranges[i].header.frame_id.c_str(), rangerArray.ranges[i].range);
        }

        sonar_pub_.publish(rangerArray);
      }

      if (enable_ir_)
      {
        drrobot_X80_player::RangeArray rangerArray;
        rangerArray.ranges.resize(IR_NUM);
        for (uint32_t i = 0; i < IR_NUM; ++i)
        {
          rangerArray.ranges[i].header.stamp = ros::Time::now();
          rangerArray.ranges[i].header.frame_id = string("drrobot_ir_");
          rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
          rangerArray.ranges[i].range = ad2Dis(rangeSensorData_.irRangeSensor[i]);
          rangerArray.ranges[i].max_range = 0.81;
          rangerArray.ranges[i].min_range = 0.1;
          //rangerArray.ranges[i].radiation_type = drrobot_X80_player::Range::INFRARED;

          rangerArray.ranges[i].radiation_type = sensor_msgs::Range::INFRARED;
          //addRangeToCloud(rangerArray.ranges[i], pointCloud);
          //ROS_INFO("I get [%s] ROBOT_ID:[%f]",rangerArray.ranges[i].header.frame_id.c_str(), ad2Dis(rangeSensorData_.irRangeSensor[i]));
        }

        ir_pub_.publish(rangerArray);
      }

      // Publish obstacle data in cloud form:
      if (pointCloud.points.size() > 2)
      {
        pointCloud.header.stamp = ros::Time::now();
        pointCloud.header.frame_id = "/base_link";
        //ROS_INFO("I get points[%5.2f][%5.2f]",pointCloud.x,pointCloud.y);
        range_cloud_pub_.publish(pointCloud);
        // TODO: publish in topic of type pointcloud2
      }

      /*drrobot_X80_player::StandardSensor standardSensor;
              standardSensor.humanSensorData.resize(4);
              standardSensor.tiltingSensorData.resize(2);
              standardSensor.overHeatSensorData.resize(2);
              standardSensor.header.stamp = ros::Time::now();
              standardSensor.header.frame_id = string("drrobot_standardsensor");
              for (uint32_t i = 0; i < 4; i++)
                standardSensor.humanSensorData[i] = standardSensorData_.humanSensorData[i];
              for (uint32_t i = 0; i < 2; i++)
                standardSensor.tiltingSensorData[i] = standardSensorData_.tiltingSensorData[i];
              for (uint32_t i = 0; i < 2; i++)
                standardSensor.overHeatSensorData[i] = standardSensorData_.overHeatSensorData[i];

              standardSensor.thermoSensorData = standardSensorData_.thermoSensorData;

              standardSensor.boardPowerVol = (double)standardSensorData_.boardPowerVol * 9 /4095;
              standardSensor.servoPowerVol = (double)standardSensorData_.servoPowerVol * 9 /4095;

              if (robotConfig1_.boardType != Jaguar)
              {
                standardSensor.motorPowerVol = (double)standardSensorData_.motorPowerVol * 24 /4095;
              }
              else
              {
                standardSensor.motorPowerVol = (double)standardSensorData_.motorPowerVol * 34.498 /4095;
              }
              standardSensor.refVol = (double)standardSensorData_.refVol / 4095 * 6;
              standardSensor.potVol = (double)standardSensorData_.potVol / 4095 * 6;
              standardSensor_pub_.publish(standardSensor);

              drrobot_X80_player::CustomSensor customSensor;
              customSensor.customADData.resize(8);
              customSensor.header.stamp = ros::Time::now();
              customSensor.header.frame_id = string("drrobot_customsensor");

              for (uint32_t i = 0; i < 8; i ++)
              {
                customSensor.customADData[i] = customSensorData_.customADData[i];
              }
              customSensor.customIO = (uint8_t)(customSensorData_.customIO & 0xff);
              customSensor_pub_.publish(customSensor);*/
    }

    // Publish the odometry in any case:
    publishOdometry(motorInfoArray);
  }

private:
  DrRobotMotionSensorDriver *drrobotMotionDriver_;
  DrRobotMotionSensorDriver *drrobotPowerDriver_;
  struct DrRobotMotionConfig robotConfig1_;
  struct DrRobotMotionConfig robotConfig2_;
  struct MotorSensorData motorSensorData_;
  struct RangeSensorData rangeSensorData_;
  struct PowerSensorData powerSensorData_;
  struct StandardSensorData standardSensorData_;
  struct CustomSensorData customSensorData_;

  std::string odom_frame_id_;
  std::string robotType_;
  std::string robotID_;
  std::string robotIP_;
  std::string robotCommMethod_;
  std::string robotSerialPort_;
  bool enable_ir_;
  bool enable_sonar_;
  int commPortNum_;
  int encoderOneCircleCnt_;
  double wheelDis_;
  double wheelRadius_;
  int motorDir_;
  double minSpeed_;
  double maxSpeed_;

  float m_x;
  float m_y;
  float m_theta;

  int cntNum_;
  double ad2Dis(int adValue)
  {
    double temp = 0;
    double irad2Dis = 0;

    if (adValue <= 0)
      temp = -1;
    else
      temp = 21.6 / ((double)adValue * 3 / 4096 - 0.17);

    if ((temp > 80) || (temp < 0))
    {
      irad2Dis = 0.81;
    }
    else if ((temp < 10) && (temp > 0))
    {
      irad2Dis = 0.09;
    }
    else
      irad2Dis = temp / 100;
    return irad2Dis;
  }

  // Convert encoder ticks to radians:
  double encoder2rad(int enc_value)
  {
    return (enc_value * 2 * M_PI) / encoderOneCircleCnt_;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drrobot_X80_player");

  DrRobotPlayerNode drrobotPlayer;
  ros::NodeHandle n;
  // Start up the robot
  if (drrobotPlayer.start() != 0)
  {
    exit(-1);
  }
  /////////////////////////////////////////////////////////////////

  ros::Rate loop_rate(10); //10Hz

  while (n.ok())
  {
    drrobotPlayer.doUpdate();
    ros::spinOnce();
    loop_rate.sleep();
  }
  /////////////////////////////////////////////////////////////////

  // Stop the robot
  drrobotPlayer.stop();

  return (0);
}
