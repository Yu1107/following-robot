
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <math.h>
#include <geometry_msgs/PoseArray.h>

#define PI 3.14159265

geometry_msgs::Point pdr, wp, oldpdr, oldwp, calibrate, avgpoint, SL;
std_msgs::Bool kpoint;

int i = 0, k = 0;
bool shortok = false;
bool leg = false;
double left_data = 0, right_data = 0, sum = 0;
double kinect_leg[50];
float xnum[20], ynum[20];
float m, b;

using namespace std;
float GetAngle(const geometry_msgs::Point new_k, const geometry_msgs::Point old_k,
               const geometry_msgs::Point new_p, const geometry_msgs::Point old_p);
void SEslope(float xnum[], float ynum[], int N, float &m, float &b);
class SubAndPub
{
public:
  SubAndPub()
  {
    //para_b_pub = n.advertise<geometry_msgs::Point>("/parameter/b", 1);
    para_a_pub = n.advertise<geometry_msgs::Point>("/parameter", 1);
    calipub = n.advertise<geometry_msgs::Point>("/calibration_b", 1);
    pdrsub = n.subscribe<geometry_msgs::Point>("/PDR_SL", 1, &SubAndPub::SLCallback, this);

    kinectpub = n.advertise<std_msgs::Bool>("/kinect_point", 10);

    goalsub = n.subscribe("/PDR_position", 1, &SubAndPub::pdrCallback, this);
    shortsub = n.subscribe("/short_kinect", 5, &SubAndPub::shortCallback, this);
    wpsub = n.subscribe("/wp", 10, &SubAndPub::wpCallback, this);

    Visible_pub = n.subscribe("get_a", 1, &SubAndPub::Callback, this);
    leg_pub = n.subscribe("leg_length", 1, &SubAndPub::legCallback, this);
  }
  void pdrCallback(const geometry_msgs::Point::ConstPtr &pdrmsg);
  void wpCallback(const geometry_msgs::Point::ConstPtr &wpmsg);
  void shortCallback(const std_msgs::Bool::ConstPtr &shortmsg);
  void Callback(const std_msgs::Bool::ConstPtr &msg);
  void legCallback(const geometry_msgs::Point::ConstPtr &msg);
  void SLCallback(const geometry_msgs::Point::ConstPtr &SLmsg);

private:
  ros::NodeHandle n;
  ros::Publisher calipub, para_a_pub, para_b_pub, kinectpub;
  ros::Subscriber goalsub, wpsub, shortsub, Visible_pub, leg_pub, pdrsub;

  // Frame broadcaster
  tf::TransformBroadcaster tfBroadcast_;
};
void SubAndPub::SLCallback(const geometry_msgs::Point::ConstPtr &SLmsg)
{
  SL.x = SLmsg->x; //dH
}
void SubAndPub::Callback(const std_msgs::Bool::ConstPtr &msg)
{
  leg = msg->data;
  ROS_INFO("Now : %d", leg);
}
void SubAndPub::legCallback(const geometry_msgs::Point::ConstPtr &msg)
{
  i += 1;
  left_data += msg->x;
  right_data += msg->y;
  if (i < 50)
  {

    kinect_leg[i] = double(left_data + right_data) / i / 2;

    ROS_INFO_STREAM("Now " << i << " length: " << (kinect_leg[i] / 1000));
    sum += kinect_leg[i] / 1000;
  }
  else if (i == 50)
  {
    geometry_msgs::Point a;
    ROS_INFO_STREAM("mean leg length: " << (sum / i));
    a.y = sum / i;
    float actual_leg = (sum / i) + (sum / i) * 0.85;
    ROS_INFO_STREAM("actual leg length: " << actual_leg);

    float H = 0.9;
    if (actual_leg > H + 0.2) //190
    {
      a.x = 0.02371;

      ROS_INFO_STREAM("parameter a: " << a.x);
    }
    else if (actual_leg > H) //180
    {
      a.x = 0.02137;

      ROS_INFO_STREAM("a : " << a.x);
    }
    else if (actual_leg > H - 0.2) //170
    {
      a.x = 0.02058;

      ROS_INFO_STREAM("a : " << a.x);
    }
    para_a_pub.publish(a);
  }
}

void SubAndPub::shortCallback(const std_msgs::Bool::ConstPtr &shortmsg)
{

  shortok = shortmsg->data;
}
void SubAndPub::wpCallback(const geometry_msgs::Point::ConstPtr &wpmsg)
{
  wp.x = wpmsg->x;
  wp.y = wpmsg->y;

  kpoint.data = false;
  kinectpub.publish(kpoint);
  //ROS_INFO_STREAM("Now wp " << wp.x <<", "<< wp.y );
}

void SubAndPub::pdrCallback(const geometry_msgs::Point::ConstPtr &pdrmsg)
{

  pdr.x = pdrmsg->x;
  pdr.y = pdrmsg->y;
  pdr.z = pdrmsg->z;
  //ROS_INFO_STREAM("Now PDR " << pdr.x <<", "<< pdr.y <<", "<< pdr.z);

  if (pdr.z == 0)
  {
    //ROS_INFO_STREAM(" kinect endpoint");
    oldwp.x = wp.x;
    oldwp.y = wp.y;
  }
  else if (pdr.x != oldpdr.x || pdr.y != oldpdr.y)
  {
    //ROS_INFO_STREAM("Now  old(" << oldwp.x << ", " << oldwp.y << ")new(" << wp.x << ", " << wp.y << ")");
    k = k + 1;

    kpoint.data = true;
    kinectpub.publish(kpoint);

    //kinect detcet person to odom
    calibrate.x = sqrt(pow(wp.x - oldwp.x, 2) + pow(wp.y - oldwp.y, 2)); //kinect between two point distance
    calibrate.y = pdr.z;                                                 //step length
    //ROS_INFO_STREAM("step " << k << " ,kinect_z( " << calibrate.x << " ),PDR_z( " << calibrate.y << " ),error " << calibrate.x - calibrate.y << ").");

    /*if (abs(calibrate.z) > 1.5)
    {
      miss++;
      ROS_INFO_STREAM("missing step :" << miss);

      //return;
    }
    if (k > 1)
    {
      sum_dist[k] = calibrate.z;
      ROS_INFO_STREAM(k << " sum_error:" << sum_dist[k] / k << " ,now_error:" << sum_dist[k]);
    }*/

    /*float diff_angle = GetAngle(wp, oldwp, pdr, oldpdr);
    ROS_INFO_STREAM("angle = " << diff_angle);

    calibrate.z = diff_angle;*/

    calipub.publish(calibrate);
    xnum[k] = SL.x;
    ynum[k] = calibrate.x;
    SEslope(xnum, ynum, k, m, b);

    geometry_msgs::Point se_slope;
    se_slope.x = m;
    se_slope.y = b;
    para_a_pub.publish(se_slope);

    oldwp.x = wp.x;
    oldwp.y = wp.y;
    oldpdr.x = pdr.x;
    oldpdr.y = pdr.y;
  }
}

/*float GetAngle(const geometry_msgs::Point new_k, const geometry_msgs::Point old_k,
               const geometry_msgs::Point new_p, const geometry_msgs::Point old_p)
{
  float v1x = new_k.x - old_k.x;
  float v1y = new_k.y - old_k.y;
  float v2x = new_p.x - old_p.x;
  float v2y = new_p.y - old_p.y;
  cout << v1x << ", " << v1y << ", " << v2x << ", " << v2y << endl;

  double d1 = (v1x * v2x) + (v1y * v2y);
  double d2 = sqrt(pow(v2x, 2) + pow(v2y, 2)) * sqrt(pow(v1x, 2) + pow(v1y, 2));
  float angle = acos(d1 / d2) * (180 / PI);

  return angle;
}*/

void SEslope(float xnum[], float ynum[], int N, float &m, float &b)
{
  int i;
  float xsum = 0.0, ysum = 0.0, xybar = 0.0, xs = 0.0, xy = 0.0, x = 0.0;
  float xbar, ybar;
  //ROS_INFO_STREAM(N << " X= " << xnum[N] << " ,Y= " << ynum[N]);
  try
  {
    for (i = 0; i < N; i++)
    {
      xsum += xnum[i];
      ysum += ynum[i];
      //xy = xnum[i] * ynum[i];
      //xybar += xy;
      xybar += xnum[i] * ynum[i];

      //x = pow(xnum[i], 2);
      //xs += x;
      xs += xnum[i] * xnum[i];
    }
  }
  catch (...)
  {
    cout << "no data input" << endl;
  }
  xbar = xsum / N;
  ybar = ysum / N;
  xybar = xybar / N;
  xs = xs / N;
  //cout << N << ".xbar(" << xbar << "), ybar(" << ybar << "), xybar(" << xybar << "), xs(" << xs << ")." << endl;
  m = (xbar * ybar - xybar) / (xbar * xbar - xs);
  b = ybar - m * xbar;
  ROS_INFO_STREAM(k << " m= " << m << " ,b= " << b);
  //return m, b;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_parameter");
  cout << "calibrate..." << endl;

  SubAndPub substerAndPubster;

  ros::spin();

  return 0;
}
