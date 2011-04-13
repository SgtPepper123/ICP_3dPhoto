#include "kinect_icp/icp_core.h"
#include <Eigen/Dense>

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

IcpCore::IcpCore(ros::Publisher publisher) : publisher_(publisher), cloud1_(NULL), cloud2_(NULL), algorithm_(NULL)
{
  Clouds_.reserve(1000);
}

void IcpCore::registerCloud(const PCloud::ConstPtr& new_point_cloud)
{ 
  ROS_DEBUG("Received Point Cloud");

  if(!algorithm_)
  {
    if (!cloud1_) {
      cloud1_ = new PCloud(*new_point_cloud);
      return;
    }

    cloud2_ = new PCloud(*new_point_cloud);

    BOOST_FOREACH (pcl::PointXYZRGB& pt, cloud2_->points) {
      pt.x += 0.1;
    }  
 
    algorithm_ = new IcpLocal(cloud1_,cloud2_);
  
  }

  //algorithm.TestMinimizeTranslate();
  
  algorithm_->Compute();

  RGBValue color;
  //color.float_value = pt.rgb;
  color.Red = 0;
  color.Green = 0;
  color.Blue = 255;

  Eigen::Matrix4f mat = algorithm_->GetTransformation();
  BOOST_FOREACH (pcl::PointXYZRGB& pt, cloud1_->points) {
    Eigen::Vector4f pnt(pt.x,pt.y,pt.z,1.0);
    pnt = mat * pnt;
    pt.x = pnt[0];
    pt.y = pnt[1];
    pt.z = pnt[2];    
    pt.rgb = color.float_value;
  }
  
  color.Blue = 0;
  color.Red = 255;
  
  BOOST_FOREACH (pcl::PointXYZRGB& pt, cloud2_->points) {
    pt.rgb = color.float_value;
  }

  PCloud OutCloud(*cloud1_);
  
  OutCloud += *cloud2_;

  publisher_.publish(OutCloud);

  /*delete cloud1_;
  delete cloud2_;
  
  cloud1_ = NULL;
  cloud2_ = NULL;*/
}
