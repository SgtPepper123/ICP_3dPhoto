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

IcpCore::IcpCore(ros::Publisher publisher) : singleMerge_(true), publisher_(publisher), outCloud_(NULL), cloud1_(NULL), cloud2_(NULL), algorithm_(NULL)
{
  Clouds_.reserve(1000);
}

void IcpCore::registerCloud(const PCloud::ConstPtr& new_point_cloud)
{ 
  ROS_DEBUG("Received Point Cloud");
  
  if(singleMerge_){
    if(!algorithm_)
    {
      if (!cloud1_) {
        cloud1_ = new PCloud(*new_point_cloud);
        return;
      }

      cloud2_ = new PCloud(*new_point_cloud);
   
      algorithm_ = new IcpLocal(cloud1_,cloud2_);
    }
      
    if(outCloud_)
    {
      delete outCloud_;
      outCloud_ = NULL;
    }
    
    outCloud_ = new PCloud(*cloud1_);
  }else
  {
      if (!cloud1_) {
        cloud1_ = new PCloud(*new_point_cloud);
        outCloud_ = new PCloud(*new_point_cloud);
        return;
      }
      
      if(cloud2_)
      {
        delete cloud2_;
        cloud2_ = NULL;
      }
      
      cloud2_ = cloud1_;
      cloud1_ = new PCloud(*new_point_cloud);

      if(algorithm_)
      {
        delete algorithm_;
      }
      algorithm_ = new IcpLocal(cloud1_,cloud2_);
      
      algorithm_->SetMaxIterations(200);
  }

  //algorithm.TestMinimizeTranslate();
  
  algorithm_->Compute();

  if(singleMerge_)
  {
    RGBValue color;
    //color.float_value = pt.rgb;
    color.Red = 0;
    color.Green = 0;
    color.Blue = 255;

    Eigen::Matrix4f mat = algorithm_->GetTransformation();
    BOOST_FOREACH (pcl::PointXYZRGB& pt, outCloud_->points) {
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
    
    *outCloud_ += *cloud2_;
    
    algorithm_->SetMaxIterations(3);

  }else
  {
    Eigen::Matrix4f mat = algorithm_->GetTransformation();
    BOOST_FOREACH (pcl::PointXYZRGB& pt, cloud1_->points) {
      Eigen::Vector4f pnt(pt.x,pt.y,pt.z,1.0);
      pnt = mat * pnt;
      pt.x = pnt[0];
      pt.y = pnt[1];
      pt.z = pnt[2];    
    }
    
    *outCloud_ += *cloud1_;
    
  }

  publisher_.publish(*outCloud_);
  
}
