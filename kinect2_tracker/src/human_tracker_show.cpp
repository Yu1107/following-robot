#include <ros/ros.h>
#include <iostream>
#include <ros/console.h>

// OpenCV Header
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include </home/lab5254/vision_opencv/cv_bridge/include/cv_bridge/cv_bridge.h>

// o1. OpenNI Header
#include <OpenNI.h> // /usr/include/openni2
// n1. NiTE Header
#include <NiTE.h>

#include "geometry_msgs/Point.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <camera_info_manager/camera_info_manager.h>

// namespace
using namespace std;
using namespace openni;
using namespace nite;

float meter = 1000.0;
bool following = false;
float rotation = -14.82 / 57.3;
int Mirroring = -1;
int mancount;
bool found = false;

geometry_msgs::Point position, realPos, PDR, wp, real_pre_pos, kalpos;
cv::Point2f aPoint[5];
std_msgs::Bool goalok, shortok;
geometry_msgs::PointStamped laser_point, base_point;

float Length(const nite::Point3f p1, const nite::Point3f p2)
{
    return sqrt(
        pow(p1.x - p2.x, 2) +
        pow(p1.y - p2.y, 2) +
        pow(p1.z - p2.z, 2));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracker_show");
    ros::NodeHandle nh, nh_priv("~");

    // Frame broadcaster
    tf::TransformBroadcaster tfBroadcast_;
    tf::TransformListener listener(ros::Duration(10));

    //ros Publisher
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::Publisher position_1_pub = nh.advertise<geometry_msgs::Point>("/tracker/position", 1);
    ros::Publisher kalman_pub = nh.advertise<geometry_msgs::Point>("/tracker/kalman_position", 1);

    ros::Publisher body_tracking_position_pub = nh.advertise<geometry_msgs::Point>("tracker_body_position", 1);
    ros::Publisher PDR_pub = nh.advertise<geometry_msgs::Point>("PDR_position", 1);

    ros::Publisher wp_pub = nh.advertise<geometry_msgs::Point>("/wp", 1);
    ros::Publisher leg_pub = nh.advertise<geometry_msgs::Point>("/leg_length", 1);
    //ros::Publisher goal_pub = nh.advertise<std_msgs::Bool>("/start_IMU", 1);
    ros::Publisher short_pub = nh.advertise<std_msgs::Bool>("/short_kinect", 1);

    image_transport::ImageTransport it_(nh);
    image_transport::Publisher image_pub_ = it_.advertise("/kinect_rgb", 1);
    //image_transport::Publisher depth_pub_ = it_.advertise("/kinect_depth", 1);
    //image_transport::Publisher color_pub_ = it_.advertise("/kinect_color", 1);

    // o2. Initial OpenNI
    OpenNI ::initialize();

    // o3. Open Device
    Device mDevice;
    mDevice.open(ANY_DEVICE);

    // create depth stream
    VideoStream mDepthStream;
    mDepthStream.create(mDevice, SENSOR_DEPTH);

    // o4a. set video mode
    VideoMode mDMode;
    mDMode.setResolution(640, 480);
    mDMode.setFps(30);
    mDMode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
    mDepthStream.setVideoMode(mDMode);
    mDepthStream.setMirroringEnabled(true);

    int iMaxDepth = mDepthStream.getMaxPixelValue();

    // o5. Create color stream
    VideoStream mColorStream;
    mColorStream.create(mDevice, SENSOR_COLOR);

    // o5a. set video mode
    VideoMode mCMode;
    mCMode.setResolution(640, 480);
    mCMode.setFps(30);
    mCMode.setPixelFormat(PIXEL_FORMAT_RGB888);
    mColorStream.setVideoMode(mCMode);
    mColorStream.setMirroringEnabled(true);

    // o6. image registration
    mDevice.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    mDevice.setDepthColorSyncEnabled(true);
    // Initial NiTE
    NiTE::initialize();

    // n3. create user tracker
    UserTracker mUserTracker;
    mUserTracker.create(&mDevice);

    // p1. start
    mColorStream.start();
    mDepthStream.start();
    //cv::namedWindow("User Image", 1);
    //cv::moveWindow("User Image", 200, 700);

    // >>>> Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;

    unsigned int type = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

    cv::Mat state(stateSize, 1, type); // [x,y,v_x,v_y,w,h]
    cv::Mat meas(measSize, 1, type);   // [z_x,z_y,z_w,z_h]

    //cv::Mat procNoise(stateSize, 1, type)
    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    cv::setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    // Process Noise Covariance Matrix Q 预测（过程）噪声方差
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 1e-2; //5.0f
    kf.processNoiseCov.at<float>(21) = 1e-2; //5.0f
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;

    // Measures Noise Covariance Matrix R 测量（观测）噪声方差
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    //cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(5.0f));
    double ticks = 0;

    ros::Rate r(50);
    while (ros::ok())
    {
        double precTick = ticks;
        ticks = (double)cv::getTickCount();

        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

        // main loop
        // p2. prepare background
        cv::Mat cImageBGR;

        // p2a. get color frame,depth frame
        VideoFrameRef mColorFrame, mDepthFrame;
        mColorStream.readFrame(&mColorFrame);
        //mDepthStream.readFrame(&mDepthFrame);

        // p2b. convert data to OpenCV format
        const cv::Mat mImageRGB(mColorFrame.getHeight(), mColorFrame.getWidth(),
                                CV_8UC3, (void *)mColorFrame.getData());

        // p2c. convert form RGB to BGR
        cv::cvtColor(mImageRGB, cImageBGR, CV_RGB2BGR);
        mColorFrame.release();
        //cv::imshow( "Color Image", cImageBGR );
        //sensor_msgs::ImagePtr color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cImageBGR).toImageMsg(); //CV_8UC3, color image with blue-green-red color order
        //color_pub_.publish(color_msg);

        // convert data to OpenCV format
        /*const cv::Mat mImageDepth(
            mDepthFrame.getHeight(), mDepthFrame.getWidth(),
            CV_16UC1, (void *)mDepthFrame.getData());

        // re-map depth data [0,Max] to [0,255]
        cv::Mat mScaledDepth;
        mImageDepth.convertTo(mScaledDepth, CV_8U, 255.0 / iMaxDepth);*/

        UserTrackerFrameRef mUserFrame;

        mUserTracker.readFrame(&mUserFrame);
        mUserTracker.setSkeletonSmoothingFactor(0.1f);

        // p4. get all users data
        const nite::Array<UserData> &aUsers = mUserFrame.getUsers();

        //ROS_INFO_STREAM("Found  user" << aUsers.getSize());
        mancount = aUsers.getSize();

        // does same tasks for every user
        // If you want the task is different for each user, the task should related to i
        for (int i = 0; i < mancount; ++i)
        {
            // assign an id for i-th user
            const UserData &rUser = aUsers[i];

            //If we don't request this for each user, we won't have any data about their skeleton and skeleton joints.
            if (mUserTracker.startSkeletonTracking(rUser.getId()))
                continue;

            // get user skeleton
            const Skeleton &rSkeleton = rUser.getSkeleton();

            printf("User #%d %s \r\n", rUser.getId(), (rUser.isVisible()) ? "is Visible" : "is not Visible");
            if (!rUser.isVisible() && following == true)
            {
                found = false;
                following = false;
                shortok.data = true;
                short_pub.publish(shortok); //publish -> marker.cpp

                ROS_INFO_STREAM("Robot stop movement");
                geometry_msgs::Twist cmd;
                cmd.linear.x = 0;
                cmd.angular.z = 0;
                cmd_pub.publish(cmd); //publish -> drrobot_player.cpp

                //vanish point
                //PDR.x = wp.x;
                //PDR.y = wp.y;
                PDR_pub.publish(wp); //publish -> marker.cpp
                wp_pub.publish(wp);  //publish -> gotogoal.py
                cout << "User " << rUser.getId() << "(" << PDR.x << "," << PDR.y << ") Out of Scene." << endl;

                mancount = 0;
                break;
            }
            if (rSkeleton.getState() == nite::SKELETON_TRACKED) //&& shortok.data == false)
            {
                // get joints
                const nite::SkeletonJoint &rmass = rSkeleton.getJoint(nite::JOINT_TORSO);
                const nite::Point3f &Position = rmass.getPosition();
                const nite::Point3f &RHPosition = rSkeleton.getJoint(nite::JOINT_RIGHT_HIP).getPosition();
                const nite::Point3f &LHPosition = rSkeleton.getJoint(nite::JOINT_LEFT_HIP).getPosition();
                const nite::Point3f &LKPosition = rSkeleton.getJoint(nite::JOINT_LEFT_KNEE).getPosition();
                const nite::Point3f &RKPosition = rSkeleton.getJoint(nite::JOINT_RIGHT_KNEE).getPosition();
                const nite::Point3f &LFPosition = rSkeleton.getJoint(nite::JOINT_LEFT_FOOT).getPosition();
                const nite::Point3f &RFPosition = rSkeleton.getJoint(nite::JOINT_RIGHT_FOOT).getPosition();

                //cout << Length(RHPosition, RKPosition) + Length(RFPosition, RKPosition) << "," << Length(LHPosition, LKPosition) + Length(LFPosition, LKPosition) << endl;
                geometry_msgs::Point leg_info;
                leg_info.x = Length(RHPosition, RKPosition) + Length(RFPosition, RKPosition);
                leg_info.y = Length(LHPosition, LKPosition) + Length(LFPosition, LKPosition);
                leg_pub.publish(leg_info);
                if (found)
                {
                    // >>>> Matrix A
                    kf.transitionMatrix.at<float>(2) = dT;
                    kf.transitionMatrix.at<float>(9) = dT;
                    // <<<< Matrix A

                    //cout << "dT:" << endl
                    //<< dT << endl;

                    state = kf.predict();
                    //cout << "State post:" << endl
                    //<< state << endl;

                    /*cv::Rect predRect;
                    predRect.width = state.at<float>(4);
                    predRect.height = state.at<float>(5);
                    predRect.x = state.at<float>(0) - predRect.width / 2;
                    predRect.y = state.at<float>(1) - predRect.height / 2;
                    cv::rectangle(cImageBGR, predRect, CV_RGB(255, 0, 0), 2);*/

                    cv::Point center;
                    center.x = state.at<float>(0); //kinect_z
                    center.y = state.at<float>(1); //kinect_x
                    //cv::circle(cImageBGR, center, 10, CV_RGB(255, 0, 0), -1);
                    kalpos.x = center.x;
                    kalpos.y = center.y;
                    kalman_pub.publish(kalpos);
                }
                if (rmass.getPositionConfidence() > 0.1)
                {
                    following = true;
                    shortok.data = false;
                    short_pub.publish(shortok); //publish -> marker.cpp

                    position.x = Position.x / meter + 0.07;
                    position.y = (sin(rotation) * Position.z / meter) + (cos(rotation) * Position.y / meter);
                    position.z = (cos(rotation) * Position.z / meter) - (sin(rotation) * Position.y / meter);
                    //ROS_INFO("Now user(%5.2f),past user(%5.2f)", position.z, real_pre_pos.z);

                    cout << "x:" << rUser.getCenterOfMass().x << " y:" << rUser.getCenterOfMass().y << " z:" << rUser.getCenterOfMass().z << endl;

                    position.x = position.x * rmass.getPositionConfidence() + real_pre_pos.x * (1 - rmass.getPositionConfidence());
                    position.z = position.z * rmass.getPositionConfidence() + real_pre_pos.z * (1 - rmass.getPositionConfidence());
                    position_1_pub.publish(position); //publish -> skeleton_follower.py

                    /*if (position.z > 3.75)
                    {
                        goalok.data = true;
                        goal_pub.publish(goalok);
                    }*/

                    float pre_distance = sqrt(pow(position.x - real_pre_pos.x, 2) + pow(position.y - real_pre_pos.y, 2));
                    cout << pre_distance << endl;
                    if (pre_distance > 0.5)
                    {
                        ROS_INFO("error target(%5.2f)", pre_distance);
                        break;
                    }
                    //convert joint position to 3D
                    realPos.x = position.z;
                    realPos.y = atan(Mirroring * position.x / position.z);
                    body_tracking_position_pub.publish(realPos); //publish -> marker.cpp
                    ROS_INFO("Now tracking user %d(%5.2f,%5.2f)-%5.2f", rUser.getId(), realPos.x, realPos.y * 57.3, rmass.getPositionConfidence());

                    // Publish the joints(torso) over the TF stream
                    tf::Vector3 currentVec3 = tf::Vector3(realPos.x, realPos.y, 0);
                    tf::Transform transform;
                    tf::Quaternion q;
                    q.setRPY(0, 0, 0);
                    transform.setOrigin(currentVec3);
                    transform.setRotation(q);
                    tfBroadcast_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "kinect_link", "kinect_body"));

                    laser_point.header.frame_id = "kinect_link";
                    laser_point.header.stamp = ros::Time();
                    laser_point.point.x = realPos.x; //設置相對於laser_link座標系的座標
                    laser_point.point.y = realPos.y;
                    try
                    {
                        listener.transformPoint("odom", laser_point, base_point);
                        //ROS_INFO("kinect_robot:(%.2f, %.2f) ---> kinect_odom:(%.2f, %.2f). ",laser_point.point.x, laser_point.point.y, base_point.point.x, base_point.point.y);

                        wp.x = base_point.point.x;
                        wp.y = base_point.point.y;
                        wp_pub.publish(wp);
                    }
                    catch (tf::TransformException &ex)
                    {
                        ROS_ERROR("Received an exception trying to transform a point form \"base_laser\" to \"base_link\": %s", ex.what());
                    }
                }

                //  build joints array
                SkeletonJoint aJoints[5];
                aJoints[0] = rSkeleton.getJoint(JOINT_LEFT_SHOULDER);
                aJoints[1] = rSkeleton.getJoint(JOINT_RIGHT_SHOULDER);
                aJoints[2] = rSkeleton.getJoint(JOINT_LEFT_HIP);
                aJoints[3] = rSkeleton.getJoint(JOINT_RIGHT_HIP);
                aJoints[4] = rSkeleton.getJoint(JOINT_TORSO);

                // convert joint position to image
                for (int s = 0; s < 5; ++s)
                {
                    const Point3f &rPos = aJoints[s].getPosition();
                    mUserTracker.convertJointCoordinatesToDepth(
                        rPos.x, rPos.y, rPos.z,
                        &(aPoint[s].x), &(aPoint[s].y));
                }

                // draw users skeleton
                if (rmass.getPositionConfidence() > 0.1)
                {
                    cv::circle(cImageBGR, aPoint[4], 5, cv::Scalar(0, 255, 0), -1);
                    cv::rectangle(cImageBGR, aPoint[0], aPoint[3], cv::Scalar(0, 255, 0), 3.5); //now frame
                }

                meas.at<float>(0) = wp.x * 100;
                meas.at<float>(1) = wp.y * 100;
                meas.at<float>(2) = 0;
                meas.at<float>(3) = 0;

                if (!found) // First detection!
                {
                    // >>>> Initialization
                    kf.errorCovPre.at<float>(0) = 1; // px
                    kf.errorCovPre.at<float>(7) = 1; // px
                    kf.errorCovPre.at<float>(14) = 1;
                    kf.errorCovPre.at<float>(21) = 1;
                    kf.errorCovPre.at<float>(28) = 1; // px
                    kf.errorCovPre.at<float>(35) = 1; // px

                    state.at<float>(0) = meas.at<float>(0);
                    state.at<float>(1) = meas.at<float>(1);
                    state.at<float>(2) = 0;
                    state.at<float>(3) = 0;
                    state.at<float>(4) = meas.at<float>(2);
                    state.at<float>(5) = meas.at<float>(3);
                    // <<<< Initialization

                    kf.statePost = state;

                    found = true;
                }
                else
                    kf.correct(meas); // Kalman Correction

                //cout << "Measure matrix:" << endl
                //<< meas << endl;
            }

            real_pre_pos.x = position.x;
            real_pre_pos.y = position.y;
            real_pre_pos.z = position.z;

            break; // only read first user

        } //for

        //////////////////////  show image
        //cv::imshow("User Image", cImageBGR);

        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cImageBGR).toImageMsg(); //CV_8UC3, color image with blue-green-red color order
        image_pub_.publish(image_msg);
        //sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mScaledDepth).toImageMsg(); // CV_8UC1, grayscale image
        //depth_pub_.publish(depth_msg);

        //  check keyboard
        if (cv ::waitKey(1) == 'q')
            ros::spinOnce();
        r.sleep();
    }

    // stop
    mUserTracker.destroy();
    mColorStream.destroy();
    mDepthStream.destroy();
    mDevice.close();
    NiTE ::shutdown();
    OpenNI ::shutdown();

    return 0;
}
