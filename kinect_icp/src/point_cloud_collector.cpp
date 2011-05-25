#include "ros/ros.h"
#include "kinect_icp/icp_core.h"
#include <boost/bind.hpp>

int main(int argc, char **argv)
{
  //intialize some ros stuff
  ros::init(argc, argv, "point_cloud_collector");

  //create ros node to be able to send/receive messages
  ros::NodeHandle n;

  // Create a Publisher for the final, merged images
  ros::Publisher merged_pub = n.advertise< kinect_icp::PCloud const >("merged_clouds", 1);

  //entrypoint of the Icp Algorithm
  kinect_icp::IcpCore core(merged_pub);

  //set in which algorithm should run
  //core.singleMerge_ = true;

  //create function pointer to registerCloud function in IcpCore
  //then subscribe it with the rosout node, so all published pointclouds will be directed to the algorithm.
  boost::function< void(const boost::shared_ptr< kinect_icp::PCloud const > &) > func = boost::bind(&kinect_icp::IcpCore::registerHashCloud, boost::ref(core), _1);
  //boost::function< void(const boost::shared_ptr< kinect_icp::PCloud const > &)> func = boost::bind(&kinect_icp::IcpCore::visualizeNormals,boost::ref(core),_1);
  //boost::function< void(const boost::shared_ptr< kinect_icp::PCloud const > &)> func = boost::bind(&kinect_icp::IcpCore::generateGroundTruth,boost::ref(core),_1);
 // boost::function< void(const boost::shared_ptr< kinect_icp::PCloud const > &)> func = boost::bind(&kinect_icp::IcpCore::tuneParameters,boost::ref(core),_1);

  ros::Subscriber sub = n.subscribe("/camera/rgb/points", 1, func);

  //wait for messages till canceled
  ros::spin();

  return 0;
}
