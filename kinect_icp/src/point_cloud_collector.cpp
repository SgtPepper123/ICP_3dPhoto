#include "ros/ros.h"
#include "kinect_icp/icp_core.h"
#include <boost/bind.hpp>

int main(int argc, char **argv) {
  //intialize some ros stuff
  ros::init(argc, argv, "point_cloud_collector");

  //create ros node to be able to send/receive messages
  ros::NodeHandle n;

  //entrypoint of the Icp Algorithm
  kinect_icp::IcpCore core;

  //create function pointer to registerCloud function in IcpCore 
  //then subscribe it with the rosout node, so all published pointclouds will be directed to the algorithm.
  boost::function< void(const boost::shared_ptr< kinect_icp::PCloud const > &)> func = boost::bind(&kinect_icp::IcpCore::registerCloud,boost::ref(core),_1);
  ros::Subscriber sub = n.subscribe("/camera/rgb/points", 1000, func);
	
  //wait for messages till canceled 
  ros::spin();

  return 0;
}

