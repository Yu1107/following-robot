#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Range.h"
#include <drrobot_X80_player/RangeArray.h>
#include <std_msgs/Bool.h>


#define MIN_DIST 0.4

float sonar_obstacle[3];
float ir_obstacle[7];
bool obstacleing = false;


class SubAndPub
{
public:
    SubAndPub() {
        pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

        //ir_sub = n.subscribe("/drrobot_ir",10, &SubAndPub::cmdVelCallback, this);
        sonar_sub = n.subscribe("/drrobot_sonar",10, &SubAndPub::Callback, this);
	    start_sub = n.subscribe("/short_kinect",10, &SubAndPub::shortCallback, this);
    }
    void cmdVelCallback(const drrobot_X80_player::RangeArray::ConstPtr& ir_msg);
    void Callback(const drrobot_X80_player::RangeArray::ConstPtr& msg);
    void shortCallback(const std_msgs::Bool::ConstPtr& shortmsg);
private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub,sonar_sub,ir_sub,start_sub;

};

/*void SubAndPub::cmdVelCallback(const drrobot_X80_player::RangeArray::ConstPtr& ir_msg)
{
   geometry_msgs::Twist velMsg;
   for (int i = 5 ; i < 6; i++) {
        ir_obstacle[i] = ir_msg -> ranges[i].range;
	//ROS_INFO_STREAM("IR:" << i  << " distance "<<  ir_obstacle[i]);

        if((ir_obstacle[0] || ir_obstacle[1] ) < MIN_DIST ){
            ROS_INFO_STREAM("TURN RIGHT!!!");
            velMsg.linear.x = 0.0;
            velMsg.angular.z = -0.8;
        }
        if(ir_obstacle[5]  < MIN_DIST ){
            ROS_INFO_STREAM("forward!!!");
            velMsg.linear.x = 0.35;
            velMsg.angular.z = 0.0;
        }
        if((ir_obstacle[2] || ir_obstacle[3] || ir_obstacle[4]) < MIN_DIST ){
            ROS_INFO_STREAM("TURN LEFT!!!");
            velMsg.linear.x = 0.0;
            velMsg.angular.z = 0.8;
        }
        //pub.publish(velMsg);
   }
    
}*/

void SubAndPub::shortCallback(const std_msgs::Bool::ConstPtr& shortmsg)
{
  obstacleing = shortmsg -> data;

}

void SubAndPub::Callback(const drrobot_X80_player::RangeArray::ConstPtr& msg)
{
   geometry_msgs::Twist velMsg;
   //for (int i = 0 ; i < 3; i++) {
    // sonar_obstacle[i] = msg -> ranges[i].range;
	//ROS_INFO_STREAM("SONAR:" << i  << " distance "<<  sonar_obstacle[i]);

        //if (obstacleing == true){
        //if(sonar_obstacle[0] < MIN_DIST ){
        if(msg -> ranges[0].range < MIN_DIST ){
            ROS_INFO_STREAM("TURN RIGHT!!!");
            velMsg.linear.x = 0.0;
            velMsg.angular.z = -1;
	        pub.publish(velMsg);
        }

        //if(sonar_obstacle[1] < MIN_DIST ){
        else if(msg -> ranges[1].range < MIN_DIST ){
            ROS_INFO_STREAM("BACKWARD!!!");
            velMsg.linear.x = -0.35;
            velMsg.angular.z = 0.0;
	        pub.publish(velMsg);
        }
        //if(sonar_obstacle[2] < MIN_DIST ){
        else if(msg -> ranges[2].range < MIN_DIST ){
            ROS_INFO_STREAM("TURN LEFT!!!");
            velMsg.linear.x = 0.0;
            velMsg.angular.z = 1;
	        pub.publish(velMsg);
        }
	else {
		velMsg.linear.x = 0;
		velMsg.angular.z = 0;
		pub.publish(velMsg);
	}

            
    //}
   
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "collisionAvoidence");
    ROS_INFO_STREAM("Start to avoid obstacles");

    SubAndPub substerAndPubster;

    ros::spin();

    return 0;
}
