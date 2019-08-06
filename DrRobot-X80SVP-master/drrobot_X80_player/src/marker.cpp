//https://answers.ros.org/question/279558/visualising-the-real-time-trajectory-path-using-markers/
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/console.h>

int ii = 1, jj = 1, kk = 1, change = 0;
bool ok = false;
bool shortok = false;
geometry_msgs::Point wp_p, end_p, start_p;
geometry_msgs::PointStamped laser_point, base_point;

class SubAndPub
{
public:
  SubAndPub()
  {
    // plot kinect to odom
    personpub = n.advertise<visualization_msgs::Marker>("/kalman_marker", 1);
    personsub = n.subscribe("/tracker/kalman_position", 1, &SubAndPub::personCallback, this);

    // PDR
    goalpub = n.advertise<visualization_msgs::Marker>("/PDR_marker", 1);
    goalsub = n.subscribe("/PDR_position", 1, &SubAndPub::cmd1VelCallback, this);

    // plot kinect_position to odom
    trackerpub = n.advertise<visualization_msgs::Marker>("/tracker_marker", 1);
    trackersub = n.subscribe("wp", 1, &SubAndPub::cmd2VelCallback, this);

    startsub = n.subscribe("/kinect_point", 5, &SubAndPub::kinectCallback, this);
    shortsub = n.subscribe("/short_kinect", 1, &SubAndPub::shortCallback, this);
    //addgoalpub = n.advertise<std_msgs::Bool>("/addgoal", 1);
    addgoal_pointpub = n.advertise<geometry_msgs::Point>("/addgoal_point", 1);
  }
  void cmd1VelCallback(const geometry_msgs::Point::ConstPtr &Point_msg1);
  void cmd2VelCallback(const geometry_msgs::Point::ConstPtr &Point_msg2);
  void kinectCallback(const std_msgs::Bool::ConstPtr &msg);
  void shortCallback(const std_msgs::Bool::ConstPtr &shortmsg);
  void personCallback(const geometry_msgs::Point::ConstPtr &Point_msg3);

private:
  ros::NodeHandle n;
  ros::Publisher goalpub, trackerpub, trackerarraypub, personpub, addgoalpub, addgoal_pointpub;
  ros::Subscriber goalsub, trackersub, startsub, shortsub, personsub;

  // Frame broadcaster
  tf::TransformBroadcaster tfBroadcast_;
  tf::TransformListener listener;
};

void SubAndPub::shortCallback(const std_msgs::Bool::ConstPtr &shortmsg)
{

  shortok = shortmsg->data;
}

void SubAndPub::kinectCallback(const std_msgs::Bool::ConstPtr &msg)
{

  ok = msg->data;
}

//kalman_kinect point
void SubAndPub::personCallback(const geometry_msgs::Point::ConstPtr &Point_msg3)
{
  visualization_msgs::Marker points, line_strip, line_list;
  visualization_msgs::MarkerArray points_array;

  points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/odom";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();

  points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  points.id = kk; //still plot path rviz
  line_list.id = kk;

  points.type = visualization_msgs::Marker::POINTS;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.03;
  points.scale.y = 0.03;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_list.scale.x = 0.03;
  line_list.scale.y = 0.03;

  // Points are white
  points.color.g = 1.0;
  points.color.b = 1.0;
  points.color.r = 1.0;
  points.color.a = 1.0;

  // Line list 为红色
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  geometry_msgs::Point p;
  points.ns = line_strip.ns = kk;
  p.x = Point_msg3->x / 100;
  p.y = Point_msg3->y / 100;
  p.z = 0;

  points.points.push_back(p);
  /*line_list.points.push_back(p);
  p.z += 0.5;
  line_list.points.push_back(p);*/

  personpub.publish(points);
  //personpub.publish(line_list);

  ++kk;
}

//PDR plot
void SubAndPub::cmd1VelCallback(const geometry_msgs::Point::ConstPtr &Point_msg1)
{
  visualization_msgs::Marker points, line_strip, line_list;

  points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/odom";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();

  points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.ns = line_strip.ns = ii;

  points.scale.x = 0.05;
  points.scale.y = 0.1;

  geometry_msgs::Point p;
  p.x = Point_msg1->x;
  p.y = Point_msg1->y;
  if (shortok == true)
  {
    std_msgs::Bool addgoal_point;
    geometry_msgs::Point wp;
    // Points are purple
    points.color.r = 0.7;
    points.color.b = 1.0;
    points.color.a = 1.0;

    if (Point_msg1->z != 0)
    {
      /*points.type = visualization_msgs::Marker::ARROW;
      addgoal_point.data = true;
      addgoalpub.publish(addgoal_point);
      points.scale.x = 0.03; //shaft diameter
      points.scale.y = 0.05; //head diameter
      points.scale.z = 0.1;  //head length
      if (change == 0)
      {

        start_p.x = wp_p.x;
        start_p.y = wp_p.y;
        end_p.x = start_p.x + Point_msg1->x;
        end_p.y = start_p.y + Point_msg1->y;
        change = change + 1;

        addgoal_pointpub.publish(end_p);
      }
      else
      {
        start_p.x = end_p.x;
        start_p.y = end_p.y;
        end_p.x = start_p.x + Point_msg1->x;
        end_p.y = start_p.y + Point_msg1->y;
        addgoal_pointpub.publish(end_p);
      }

      points.points.push_back(start_p);
      points.points.push_back(end_p);
      ROS_INFO_STREAM(start_p.x << "," << start_p.y << "-->" << end_p.x << "," << end_p.y);*/

      geometry_msgs::PointStamped laser_point, base_point;
      laser_point.header.frame_id = "kinect_body"; //以vanish point 為原點 plot PDR point
      laser_point.header.stamp = ros::Time();
      laser_point.point.x = Point_msg1->x; //設置相對於laser_link座標系的座標
      laser_point.point.y = Point_msg1->y;
      try
      {
        listener.transformPoint("odom", laser_point, base_point);
        ROS_INFO("body_end:(%.2f, %.2f) ---> PDR_odom:(%.2f, %.2f). ", laser_point.point.x, laser_point.point.y, base_point.point.x, base_point.point.y);

        /*wp.x = base_point.point.x;
        wp.y = base_point.point.y;
        points.points.push_back(wp);
        addgoal_pointpub.publish(wp);*/
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("Received an exception trying to transform a point form \"base_laser\" to \"base_link\": %s", ex.what());
      }
    }
    else //vanish point (no length)
    {
      points.scale.x = 0.1;
      points.scale.y = 0.1;
      ROS_INFO_STREAM("kinect end(" << p.x << "," << p.y);
    }

    addgoal_pointpub.publish(p);
  }

  /*else
  {
    // Points are blue
    points.color.b = 1.0;
    points.color.a = 1.0;
  }*/
  points.points.push_back(p);
  goalpub.publish(points);

  ++ii;
}

//kinect to odom plot
void SubAndPub::cmd2VelCallback(const geometry_msgs::Point::ConstPtr &Point_msg2)
{
  visualization_msgs::Marker points, line_strip, line_list;
  visualization_msgs::MarkerArray points_array;

  points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/odom";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();

  points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  points.id = jj;
  line_list.id = jj;
  line_strip.id = 1;

  points.type = visualization_msgs::Marker::POINTS;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.03;
  points.scale.y = 0.03;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_list.scale.x = 0.03;
  line_list.scale.y = 0.03;

  line_strip.scale.x = 0.03;

  // Points are green
  points.color.g = 1.0;
  points.color.a = 1.0;

  // Line list 为红色
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  // Line strip 是蓝色
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;
  points.ns = line_strip.ns = jj;

  wp_p.x = Point_msg2->x;
  wp_p.y = Point_msg2->y;
  wp_p.z = 0;
  points.points.push_back(wp_p);

  /*if (shortok == true)
  {
    p.x = Point_msg2->x + 0.1;
    p.y = Point_msg2->y + 0.1;
    points.points.push_back(p);
    line_list.points.push_back(p);
    line_strip.points.push_back(p);
    p.z += 1.0;
    line_list.points.push_back(p);
    points.points.push_back(p);

    geometry_msgs::Point p1;
    p1.x = Point_msg2->x - 0.1;
    p1.y = Point_msg2->y - 0.1;
    p1.z = 0;

    line_list.points.push_back(p1);
    points.points.push_back(p1);
    line_strip.points.push_back(p1);
    p1.z += 1.0;
    line_list.points.push_back(p1);
    points.points.push_back(p1);

  }*/

  /*if (ok == true) //record kinect point
  {
    ROS_INFO_STREAM("kinect point(" << wp_p.x << "," << wp_p.y << ").");

    points.scale.x = 0.05;
    points.scale.y = 0.2;

    // Points are cyan-blue
    points.color.g = 0.7;
    points.color.b = 0.7;
    points.color.a = 1.0;
    line_list.points.push_back(wp_p);
    wp_p.z += 0.5;
    line_list.points.push_back(wp_p);
  }*/

  trackerpub.publish(points);
  trackerpub.publish(line_list);
  //trackerpub.publish(line_strip);
  ++jj;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_path");

  SubAndPub substerAndPubster;
  ros::spin();

  return 0;
}
