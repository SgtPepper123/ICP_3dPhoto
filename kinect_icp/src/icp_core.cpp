#include "kinect_icp/icp_core.h"

using namespace kinect_icp;

IcpCore::IcpCore(ros::Publisher publisher) : publisher_(publisher)
{
  m_Clouds.reserve(1000);
}

void IcpCore::registerCloud(const PCloud::ConstPtr& new_point_cloud)
{ 
  ROS_INFO("Received");
  if(Clouds_.begin()!=Clouds_.end())
  {
    IcpLocal algorithm(*(Clouds_.end()-1),new_point_cloud);
    algorithm.Compute();		
  } 
  Clouds_.push_back(new_point_cloud);

  publisher_.publish(new_point_cloud);
}
