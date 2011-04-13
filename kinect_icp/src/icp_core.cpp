#include "kinect_icp/icp_core.h"

using namespace kinect_icp;
using namespace std;

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

IcpCore::IcpCore(ros::Publisher publisher) : publisher_(publisher), cloud1_(NULL), cloud2_(NULL)
{
  Clouds_.reserve(1000);
}

void IcpCore::registerCloud(const PCloud::ConstPtr& new_point_cloud)
{ 
  ROS_DEBUG("Received Point Cloud");

  if (!cloud1_) {
    cloud1_ = new PCloud(*new_point_cloud);
    return;
  }

  cloud2_ = new PCloud(*cloud1_);

  BOOST_FOREACH (pcl::PointXYZRGB& pt, cloud2_->points) {
    pt.x += 0.1;
  }  

  IcpLocal algorithm(cloud1_,cloud2_);
  
  algorithm.TestMinimizeTranslate();
  
  /*int iterations = 1;
  do
  {
    cout << "iteration " << iterations << ":" << endl;
    algorithm.Compute();
    iterations++; 
  }while(algorithm.GetChange()>0.0001 && iterations < 10);*/

  RGBValue color;
  //color.float_value = pt.rgb;
  color.Red = 0;
  color.Green = 0;
  color.Blue = 255;

  BOOST_FOREACH (pcl::PointXYZRGB& pt, cloud1_->points) {
    pt.rgb = color.float_value;
  }
  
  color.Blue = 0;
  color.Red = 255;
  
  BOOST_FOREACH (pcl::PointXYZRGB& pt, cloud2_->points) {
    pt.rgb = color.float_value;
  }

  *cloud1_ += *cloud2_;

  publisher_.publish(*cloud1_);

  delete cloud1_;
  delete cloud2_;
  
  cloud1_ = NULL;
  cloud2_ = NULL;
}
