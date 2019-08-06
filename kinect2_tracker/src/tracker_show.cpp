#include <ros/ros.h>
#include <iostream>
#include <ros/console.h>
#include <array>

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

// namespace
using namespace std;
using namespace openni;
using namespace nite;

float meter = 1000.0;
bool stop_following = false;
bool following = false, catchbody = false;
float Gestures, beta;
float rotation = -13 / 57.3;
int Mirroring = -1;
int lost = 0, mancount = 0;
float bodies_x[3];
float bodies_y[3];
float bodies[3][3];
float d[3];

geometry_msgs::Point position, position_r, position_l, realPos, PDR, person, wp;
cv::Point2f aPoint[15];
std_msgs::Bool goalok, shortok;
geometry_msgs::PointStamped laser_point, base_point;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "tracker_show");
    ros::NodeHandle nh, nh_priv("~");

    // Frame broadcaster
    tf::TransformBroadcaster tfBroadcast_;
    tf::TransformListener listener(ros::Duration(10));

    //ros Publisher
    ros::Publisher position_1_pub = nh.advertise<geometry_msgs::Point>("/tracker/position", 1);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Publisher right_foot_pub = nh.advertise<geometry_msgs::Point>("/right_foot/position", 1);
    ros::Publisher left_foot_pub = nh.advertise<geometry_msgs::Point>("/left_foot/position", 1);
    ros::Publisher body_tracking_position_pub = nh.advertise<geometry_msgs::Point>("tracker_body_position", 1);
    ros::Publisher PDR_pub = nh.advertise<geometry_msgs::Point>("PDR_position", 1);

    //ros::Publisher person_pub = nh.advertise<geometry_msgs::Point>("/person_position", 1);
    //ros::Publisher goal_pub = nh.advertise<std_msgs::Bool>("/start_IMU", 1);
    ros::Publisher short_pub = nh.advertise<std_msgs::Bool>("/short_kinect", 1);
    ros::Publisher wp_pub = nh.advertise<geometry_msgs::Point>("/wp", 1);

    image_transport::ImageTransport it_(nh);
    image_transport::Publisher image_pub_ = it_.advertise("/kinect_rgb", 1);

    // o2. Initial OpenNI
    OpenNI ::initialize();

    // o3. Open Device
    Device mDevice;
    mDevice.open(ANY_DEVICE);
    // Listening to the device connect
    //printf("%s Opened, Completed.\r\n", mDevice.getDeviceInfo().getName());

    // create depth stream
    VideoStream mDepthStream;
    mDepthStream.create(mDevice, SENSOR_DEPTH);

    /*const openni::Array<VideoMode> *supportedVideoModes = &(mDepthStream.getSensorInfo().getSupportedVideoModes());
     * int numOfVideoModes = supportedVideoModes->getSize();
     * for (int i = 0; i < numOfVideoModes; i++){
     * VideoMode vm = (*supportedVideoModes)[i];
     * printf("%c. %dx%d at %dfps with %d format \r\n",
     * 'a' + i,
     * vm.getResolutionX(),
     * vm.getResolutionY(),
     * vm.getFps(),
     * vm.getPixelFormat());
     * }*/

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

    // Initial NiTE
    NiTE::initialize();

    // n3. create user tracker
    UserTracker mUserTracker;
    mUserTracker.create(&mDevice);
    //ros::Timer timer = nh.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
    /*HandTracker hTracker;
     * nite::Status status = hTracker.create(&mDevice);
     * hTracker.startGestureDetection(nite::GESTURE_HAND_RAISE);*/

    // create OpenCV Window
    //cv::namedWindow("User Image", 0); // the size of window can be adjusted.

    // p1. start
    mColorStream.start();
    mDepthStream.start();

    //setvbuf(stdout, NULL, _IOLBF,0);
    ros::Rate r(50);
    while (ros::ok())
    {
        // main loop
        // p2. prepare background
        cv::Mat cImageBGR, mScaledDepth;

        // p2a. get color frame,depth frame
        VideoFrameRef mColorFrame, mDepthFrame;
        mColorStream.readFrame(&mColorFrame);

        // p2b. convert data to OpenCV format
        const cv::Mat mImageRGB(mColorFrame.getHeight(), mColorFrame.getWidth(),
                                CV_8UC3, (void *)mColorFrame.getData());

        // p2c. convert form RGB to BGR
        cv::cvtColor(mImageRGB, cImageBGR, CV_RGB2BGR);
        mColorFrame.release();
        //cv::imshow( "Color Image", cImageBGR );

        //////////////////////////////////////////////// get gesture data////////////////////////////////////////////////
        /*HandTrackerFrameRef newFrame;
         * hTracker.readFrame(&newFrame);
         *
         * const nite::Array<nite::GestureData>& gestures = newFrame.getGestures();
         * for (int i = 0; i < gestures.getSize(); ++i){
         * printf("%s Gesture Detected @ %g,%g,%g - %s \r\n", (gestures[i].getType() == nite::GESTURE_CLICK) ? "Click" : ((gestures[i].getType() == nite::GESTURE_HAND_RAISE) ? "Hand Raise" : "Wave"),
         * gestures[i].getCurrentPosition().x,
         * gestures[i].getCurrentPosition().y,
         * gestures[i].getCurrentPosition().z,
         * (gestures[i].isInProgress()) ? "In Progress" :
         * ((gestures[i].isComplete()) ? "Completed" :
         * "Initializing"));
         *
         * if (gestures[0].getType() == nite::GESTURE_HAND_RAISE)
         * {
         * stop_following = true;
         * }
         * }*/

        //////////////////////////////////////////////////  get all users frame////////////////////////////////////////////////
        UserTrackerFrameRef mUserFrame;
        mUserTracker.readFrame(&mUserFrame);
        mUserTracker.setSkeletonSmoothingFactor(0.1f);

        // p4. get all users data
        const nite::Array<UserData> &aUsers = mUserFrame.getUsers();
        mancount = aUsers.getSize();

        if (mancount == 0)
        {
            catchbody = false;
        }
        if (mancount != 0)
        {
            // does same tasks for every user
            // If you want the task is different for each user, the task should related to i
            for (int i = 0; i < mancount; ++i)
            {
                // assign an id for i-th user
                const UserData &rUser = aUsers[i];
                //cout << "Value of :" << &rUser << endl;

                // p4a. check i-th user status
                if (rUser.isNew())
                {
                    // start tracking for new user
                    mUserTracker.startSkeletonTracking(rUser.getId());
                    //hTracker.startGestureDetection(nite::GESTURE_HAND_RAISE);
                    ROS_INFO_STREAM("Found a new user.");
                }

                //if (mUserTracker.startSkeletonTracking( rUser.getId () ))
                //  continue;

                else if (rUser.isVisible())
                {
                    // get user skeleton
                    const Skeleton &rSkeleton = rUser.getSkeleton();

                    if (rSkeleton.getState() == nite::SKELETON_TRACKED)
                    {
                        ROS_INFO_STREAM("Now tracking user " << rUser.getId());

                        // get joints
                        const nite::SkeletonJoint &rmass = rSkeleton.getJoint(nite::JOINT_TORSO);
                        const nite::Point3f &Position = rmass.getPosition();
                        bodies_x[i] = Position.x / meter;
                        bodies_y[i] = Position.z / meter;
                        ROS_INFO_STREAM("user: " << rUser.getId() << "(" << bodies_y[i] << "," << bodies_x[i] << ")");

                        //const nite::Point3f& HPosition = rSkeleton.getJoint(nite::JOINT_HEAD).getPosition();
                        //const nite::Point3f& NPosition = rSkeleton.getJoint(nite::JOINT_NECK).getPosition();
                        const nite::Point3f &RSPosition = rSkeleton.getJoint(nite::JOINT_LEFT_SHOULDER).getPosition();
                        const nite::Point3f &REPosition = rSkeleton.getJoint(nite::JOINT_LEFT_ELBOW).getPosition();
                        const nite::Point3f &RWPosition = rSkeleton.getJoint(nite::JOINT_LEFT_HAND).getPosition();

                        const nite::Point3f &LFPosition = rSkeleton.getJoint(nite::JOINT_LEFT_FOOT).getPosition();
                        const nite::Point3f &RFPosition = rSkeleton.getJoint(nite::JOINT_RIGHT_FOOT).getPosition();
                        /*const nite::Point3f& LFPosition = rSkeleton.getJoint(nite::JOINT_LEFT_FOOT).getPosition();*/

                        /*float uv = (RSPosition.x-REPosition.x)*(RWPosition.x-REPosition.x)+(RSPosition.y-REPosition.y)*(RWPosition.y-REPosition.y)+(RSPosition.z-REPosition.z)*(RWPosition.z-REPosition.z);
                        float u_v = sqrt(pow(RSPosition.x-REPosition.x,2)+pow(RSPosition.y-REPosition.y,2)+pow(RSPosition.z-REPosition.z,2))*sqrt(pow(RWPosition.x-REPosition.x,2)+pow(RWPosition.y-REPosition.y,2)+pow(RWPosition.z-REPosition.z,2));
                        Gestures = acos(uv/u_v)*57.3;
                        ROS_INFO("angle: %5.2f, RS(%5.2f),RW(%5.2f)",Gestures,RSPosition.z , RWPosition.z);*/

                        /*float d = sqrt(pow(HPosition.x-NPosition.x,2)+pow(HPosition.y-NPosition.y,2)+pow(HPosition.z-NPosition.z,2))/1000;
                        float e = sqrt(pow(NPosition.x-Position.x,2)+pow(NPosition.y-Position.y,2)+pow(NPosition.z-Position.z,2))/1000;

                        float f = sqrt(pow(Position.x-LHPosition.x,2)+pow(Position.y-LHPosition.y,2)+pow(Position.z-LHPosition.z,2))/1000;
                        float g = sqrt(pow(LHPosition.x-LKPosition.x,2)+pow(LHPosition.y-LKPosition.y,2)+pow(LHPosition.z-LKPosition.z,2))/1000;
                        float h = sqrt(pow(LKPosition.x-LFPosition.x,2)+pow(LKPosition.y-LFPosition.y,2)+pow(LKPosition.z-LFPosition.z,2))/1000;

                        float b = sqrt(pow(RWPosition.x-REPosition.x,2)+pow(RWPosition.y-REPosition.y,2)+pow(RWPosition.z-REPosition.z,2))/1000;
                        float a = sqrt(pow(RSPosition.x-REPosition.x,2)+pow(RSPosition.y-REPosition.y,2)+pow(RSPosition.z-REPosition.z,2))/1000;
                        float c = sqrt(pow(RWPosition.x-RSPosition.x,2)+pow(RWPosition.y-RSPosition.y,2)+pow(RWPosition.z-RSPosition.z,2))/1000; 


                        cout <<"head-neck("<< position_r.x <<") neck-torso("<< position_r.y <<") hand - SHOULDER("<< position_r.z <<")" << endl;
                        beta = acos((pow(a,2)+pow(b,2)+pow(c,2))/(2*a*b));
                        //ROS_INFO("WE(%5.2f) SE(%5.2f) WS(%5.2f),beta(%5.2f) ",b,a,c,beta);
                        cout << b <<","<< a <<","<< c <<","<< beta << endl;                    

                        if (140 < Gestures  && position_r.z > 300  && Position.z < 1.8){
                        stop_following = true;
                        following = true;
                        ROS_INFO("Gestures stop .");
                        }*/

                        //if (i == 0 && rmass.getPositionConfidence() > 0.3)
                        if (rmass.getPositionConfidence() > 0.3)
                        {
                            catchbody = true; //first time detect
                            following = true;
                            //goalok.data = false;
                            //goal_pub.publish(goalok);   //publish -> matlab

                            shortok.data = false;
                            short_pub.publish(shortok); //publish -> marker.cpp

                            position.x = Position.x / meter + 0.11;
                            position.y = (sin(rotation) * Position.z / meter) + (cos(rotation) * Position.y / meter);
                            position.z = (cos(rotation) * Position.z / meter) - (sin(rotation) * Position.y / meter);

                            position_r.x = RFPosition.x / meter;
                            position_r.y = RFPosition.y / meter;
                            position_r.z = RFPosition.z / meter;

                            position_l.x = LFPosition.x / meter;
                            position_l.y = LFPosition.y / meter;
                            position_l.z = LFPosition.z / meter;

                            right_foot_pub.publish(position_r);
                            left_foot_pub.publish(position_l);
                            position_1_pub.publish(position); //publish -> skeleton_follower.py

                            //ROS_INFO("user %d,skeleton(%5.2f,%5.2f)",rUser.getId(),position.z,position.x);

                            //convert joint position to 3D
                            realPos.x = position.z;
                            realPos.y = Mirroring * position.x;
                            realPos.z = 0;
                            body_tracking_position_pub.publish(realPos); //publish -> marker.cpp

                            // Publish the joints(torso) over the TF stream
                            tf::Vector3 currentVec3 = tf::Vector3(realPos.x, realPos.y, 0);
                            tf::Transform transform;
                            tf::Quaternion q;
                            q.setRPY(1.57, 0, 1.57);
                            //q.setRPY(0, 0, 0);

                            transform.setOrigin(currentVec3);
                            transform.setRotation(q);
                            tfBroadcast_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "kinect_link", "kinect_body"));

                            laser_point.header.frame_id = "kinect_link";
                            laser_point.header.stamp = ros::Time();
                            laser_point.point.x = realPos.x; //*cos(realPos.y);     //設置相對於laser_link座標系的座標
                            laser_point.point.y = realPos.y; //*sin(realPos.y);

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
                        else
                        {
                            ROS_INFO("detect user i = %d ", rUser.getId());
                            //break;
                        }
                        //  build joints array
                        SkeletonJoint aJoints[15];

                        aJoints[2] = rSkeleton.getJoint(JOINT_LEFT_SHOULDER);
                        aJoints[3] = rSkeleton.getJoint(JOINT_RIGHT_SHOULDER);
                        aJoints[4] = rSkeleton.getJoint(JOINT_LEFT_ELBOW);
                        aJoints[5] = rSkeleton.getJoint(JOINT_RIGHT_ELBOW);
                        aJoints[6] = rSkeleton.getJoint(JOINT_LEFT_HAND);
                        aJoints[7] = rSkeleton.getJoint(JOINT_RIGHT_HAND);
                        aJoints[8] = rSkeleton.getJoint(JOINT_TORSO);
                        aJoints[9] = rSkeleton.getJoint(JOINT_LEFT_HIP);
                        aJoints[10] = rSkeleton.getJoint(JOINT_RIGHT_HIP);
                        aJoints[11] = rSkeleton.getJoint(JOINT_LEFT_KNEE);
                        aJoints[12] = rSkeleton.getJoint(JOINT_RIGHT_KNEE);
                        aJoints[13] = rSkeleton.getJoint(JOINT_LEFT_FOOT);
                        aJoints[14] = rSkeleton.getJoint(JOINT_RIGHT_FOOT);
                        // convert joint position to image
                        for (int s = 2; s < 15; ++s)
                        {
                            const Point3f &rPos = aJoints[s].getPosition();
                            mUserTracker.convertJointCoordinatesToDepth(
                                rPos.x, rPos.y, rPos.z,
                                &(aPoint[s].x), &(aPoint[s].y));
                        }
                        cv::line(cImageBGR, aPoint[9], aPoint[11], cv::Scalar(0, ((i + 1) % 3) * 255, ((i + 1) % 2) * 255), 5);
                        cv::line(cImageBGR, aPoint[10], aPoint[12], cv::Scalar(0, ((i + 1) % 3) * 255, ((i + 1) % 2) * 255), 5);
                        cv::line(cImageBGR, aPoint[11], aPoint[13], cv::Scalar(0, ((i + 1) % 3) * 255, ((i + 1) % 2) * 255), 5);
                        cv::line(cImageBGR, aPoint[12], aPoint[14], cv::Scalar(0, ((i + 1) % 3) * 255, ((i + 1) % 2) * 255), 5);
                        // draw users skeleton
                        //if (i == 0 && rmass.getPositionConfidence() > 0.1)
                        if (rmass.getPositionConfidence() > 0.1)
                        {

                            //cv::circle(cImageBGR, aPoint[8], 10, cv::Scalar(0, 0, 255), -1);
                            cv::rectangle(cImageBGR, aPoint[2], aPoint[10], cv::Scalar(0, 255, 0), 5);
                        }
                        else if (i != 0 && rmass.getPositionConfidence() > 0.1)
                        {
                            cv::line(cImageBGR, aPoint[2], aPoint[10], cv::Scalar(0, 0, 255), 5);
                            cv::line(cImageBGR, aPoint[3], aPoint[9], cv::Scalar(0, 0, 255), 5);
                        }
                    }
                }

                else if (!rUser.isVisible() && following == true)
                {
                    //if (i < 1 && following == true)

                    following = false;
                    //goalok.data = true;
                    //goal_pub.publish(goalok);   //publish -> odom_out_and_back.py

                    shortok.data = true;        //kinect
                    short_pub.publish(shortok); //publish -> marker.cpp

                    ROS_INFO_STREAM("Robot stop movement");
                    geometry_msgs::Twist cmd;
                    //cmd.linear.x = 0;
                    //cmd.angular.z = 0;
                    //cmd_pub.publish(cmd); //publish -> drrobot_player.cpp

                    cout << "User " << rUser.getId() << " Out of Scene." << endl;
                    lost = rUser.getId();

                    PDR.x = wp.x;
                    PDR.y = wp.y;
                    PDR_pub.publish(PDR);
                    wp_pub.publish(wp);

                } //else if(!rUser.isVisible())

            } //for

            /*if (catchbody == true && mancount > 1 ){
                int change = sizeof(bodies_x) - sizeof(bodies_x[0]);
                if( change != 0 ){
                    float d[10];
                    for (int g=1; g < mancount ; g++ ){
                            d[g] = sqrt((pow(bodies_x[g]-bodies_x[0],2)) + (pow(bodies_y[g]-bodies_y[0],2)));
                             ROS_INFO("distance %d(%5.2f)",g,d[g]);
                        if  (d[g] - d[0] > 0.5 || d[g] == d[0]){
                        mancount = 0;
                        ROS_INFO_STREAM("reset---------------------------------");
                        break;
                    }
                    }
       
                        

                }

            }*/
        }

        /*int n,p,m,n_pos;
         * float d[3];
         * float Min;
         * if (following == true && mancount > 1 ){
         *
         * //float Min = bodies[0][0];
         * for (n=1;n < mancount;n++){
         *
         * d[n]=sqrt((pow(bodies_x[n]-bodies_x[0],2)) + (pow(bodies_y[n]-bodies_y[0],2)));
         *
         * }
         *
         * for (m=1; m < n; n++){
         *
         * float Min =min(d[n],d[n-1]);
         * n_pos = n;
         * }
         * cout <<  "nMin "<< n_pos <<"is :" << Min  << endl;
         * i=0;
         *
         * }*/

        /*for (int k = 1; k < 5; k++)
        { //512*424pixels
            int kk = 102.4;
            //cv::line( cImageBGR, cv::Point(k*kk,53), cv::Point(k*kk,371), cv::Scalar ( 255, 0, 0 ), 2.5 );
            cv::line(cImageBGR, cv::Point(256, 0), cv::Point(256, 480), cv::Scalar(255, 0, 0), 2.5);
        }*/

        //////////////////////  show image //////////////////////
        //cv::imshow( "User Image" , cImageBGR );
        //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", mScaledDepth).toImageMsg();  // CV_8UC1, grayscale image
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cImageBGR).toImageMsg(); //CV_8UC3, color image with blue-green-red color order
        image_pub_.publish(msg);

        //  check keyboard
        if (cv ::waitKey(1) == 'q')

            ros::spinOnce();
        r.sleep();
        //fflush(stdout);
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
