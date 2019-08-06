#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;
geometry_msgs::Point waypoint, pre_goal, newgoal;
bool ok, before, following = true, enabled;
int i = 0, k = 0;
ros::Publisher PDR_pub;

void Callback(const geometry_msgs::Point &msg)
{

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

  //wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(0.1)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  if (ok == true)
  {
    ROS_INFO_STREAM("new goal coming " << msg.x << "," << msg.y);

    following == false;
    ROS_INFO_STREAM("waypoint " << msg.x << "," << msg.y);
    /*geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "odom";
    goal.header.stamp = ros::Time::now();

    goal.pose.orientation.w = 1;
    goal.pose.position.x = msg.x;
    goal.pose.position.y = msg.y;
    PDR_pub.publish(goal);*/
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = msg.x;
    goal.target_pose.pose.position.y = msg.y;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO_STREAM("Sending goal(" << k << "):" << goal.target_pose.pose.position.x << "," << goal.target_pose.pose.position.y);

    ac.sendGoal(goal);
  }
}

/*void Callback(const geometry_msgs::Point &msg)
{

  ROS_INFO_STREAM("waypoint " << msg.x << "," << msg.y);
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

  //wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(0.5)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;

  if (ok == true)
  {
    k++;
    before = true;
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = msg.x;
    goal.target_pose.pose.position.y = msg.y;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO_STREAM("Sending goal(" << k << "):" << goal.target_pose.pose.position.x << "," << goal.target_pose.pose.position.y);

    ac.sendGoal(goal);
    //ac.waitForResult();

    //Allow 0.5 minute to get there
    bool finished_within_time = ac.waitForResult(ros::Duration(15)); //等待直到目標完成

    if (finished_within_time)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());

      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO_STREAM("Goal succeeded!");
    }
    else
    {
      ac.cancelGoal();
      ROS_INFO_STREAM("Timed out achieving goal");
      
    }
  }
}*/
void shortCallback(const std_msgs::Bool::ConstPtr &msg)
{

  ok = msg->data;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  ros::Subscriber Visible_pub = nh.subscribe("/addgoal_point", 1, Callback);
  ros::Subscriber short_pub = nh.subscribe("/short_kinect", 1, &shortCallback);
  PDR_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

  ros::Rate loop_rate(10);
  /*while (ros::ok())
  {

    ros::spinOnce();
    loop_rate.sleep();
  }*/
  ros::spin();
  return 0;
}
