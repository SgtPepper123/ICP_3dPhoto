#include "kinect_icp/icp_core.h"

using namespace kinect_icp;

IcpCore::IcpCore()
{
	m_Clouds.reserve(1000);
}

void IcpCore::registerCloud(const PCloud::ConstPtr& new_point_cloud)
{ 
	ROS_INFO("Received");
	/*if(m_Clouds.begin()!=m_Clouds().end)
	{
		//IcpLocal algortihm(m_Clouds.end()-1,new_point_cloud);
		//algorithm.compute();
		
	} 
	m_Clouds.push_back(new_point_cloud);*/
}
