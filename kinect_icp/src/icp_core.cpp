#include "kinect_icp/icp_core.h"

using namespace kinect_icp;

typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;

IcpCore::IcpCore(ros::Publisher publisher) : publisher_(publisher)
{
  Clouds_.reserve(1000);
}

void IcpCore::registerCloud(const PCloud::ConstPtr& new_point_cloud)
{ 
  ROS_INFO("Received");
  if(Clouds_.begin()!=Clouds_.end())
  {
    IcpLocal algorithm(*(Clouds_.end()-1),new_point_cloud);
    algorithm.Compute();		
  } 
  //Clouds_.push_back(new_point_cloud);

  PCloud cloud(*new_point_cloud);

  BOOST_FOREACH (pcl::PointXYZRGB& pt, cloud.points) {
    RGBValue color;
    color.float_value = pt.rgb;
    color.Red = 0;
    color.Green = 0;
    color.Blue = 255;
    //printf ("%i\n", color.Red);
    pt.rgb = color.float_value;
  }

  publisher_.publish(cloud);
}
