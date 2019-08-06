#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void PoseCallback(const geometry_msgs::Pose::ConstPtr& goal, MoveBaseClient* ac)
{
move_base_msgs::MoveBaseGoal mb_goal;

 mb_goal.target_pose.header.frame_id = "map";
 mb_goal.target_pose.header.stamp = ros::Time::now();

 geometry_msgs::Pose check;
 check.position=goal->position;
 check.orientation=goal->orientation;
 mb_goal.target_pose.pose.position.x=check.position.x;
 mb_goal.target_pose.pose.orientation.z=check.orientation.z;
 mb_goal.target_pose.pose.orientation.w = 1.0;
  ROS_INFO("The robot has reached the goal %f, %f", mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.orientation.z);

  ac->sendGoal(mb_goal);

 ac->waitForResult();

  if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)

    ROS_INFO("The robot has reached the goal");
  else

    ROS_INFO("The base failed to get to the goal");

}

int main(int argc, char** argv)

{

  ros::init(argc, argv, "move_base_client");

  ros::NodeHandle nh;

  MoveBaseClient ac("move_base", true);

 // while(!ac.waitForServer(ros::Duration(5.0)))

//{

  //ROS_INFO("Waiting for the move_base action server to come up");

 // }

 ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>("vision_coordinates/jaguar", 1, boost::bind(PoseCallback, _1, &ac));

 ros::spin();

  return 0;

}
