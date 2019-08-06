/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#include "kinect2_tracker.hpp"

int main(int argc, char** argv)
{
//initiate ros skeleton_tracker
  ros::init(argc, argv, "kinect2_tracker_node");

  k2_tracker* skeleton_tracker = new k2_tracker();

  while (ros::ok())
  {
    skeleton_tracker->spinner();
  }

  delete skeleton_tracker;

}
