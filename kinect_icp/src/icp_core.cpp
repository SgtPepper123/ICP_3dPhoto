#include "kinect_icp/icp_core.h"
#include <Eigen/Dense>

using namespace kinect_icp;
using namespace Eigen;
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

IcpCore::IcpCore(ros::Publisher publisher)
: singleMerge_(true)
, publisher_(publisher)
, outCloud_(NULL)
, cloud1_(NULL)
, cloud2_(NULL)
, algorithm_(NULL)
, lastTransformation_(Eigen::Matrix4f::Identity())
{
  Clouds_.reserve(1000);
}

void IcpCore::visualizeNormals(const PCloud::ConstPtr& new_point_cloud) {
  PCloud* cloud = new PCloud(*new_point_cloud);

  RGBValue red;
  //color.float_value = pt.rgb;
  red.Red = 255;
  red.Green = 0;
  red.Blue = 0;

  RGBValue green;
  //color.float_value = pt.rgb;
  green.Red = 0;
  green.Green = 255;
  green.Blue = 0;

  RGBValue blue;
  //color.float_value = pt.rgb;
  blue.Red = 0;
  blue.Green = 0;
  blue.Blue = 255;

/*  BOOST_FOREACH (pcl::PointXYZRGB& pt, cloud->points) {
//    Eigen::Vector4f new_point(pt.x+2,pt.y+2,pt.z+2,1.0);


//    pnt = mat * pnt;
//pt.x = pnt[0];
  //  pt.y = pnt[1];
    //pt.z = pnt[2];
    pt.rgb = blue.float_value;
//    IcpLocal::Compute
  }*/

  int width = cloud->width;
  int height = cloud->height;

  srand(42);
  for (int i=0; i<width; i++) {
    int radius = 1;

    // Select random point
    int x = (rand()%(width-2*10))+10;
    int y = (rand()%(height-2*10))+10;
    Point& pt = (*cloud)(x,y);
    if(pcl::hasValidXYZ(pt)){
      pt.rgb = red.float_value;

      for (int i=0; i<20; i++) {
        Vector3f normal;
        if (IcpLocal::ComputeNormal(x, y, normal, radius, cloud)) {
          if(normal[1] > 0)
            normal = -normal;
          pcl::PointXYZRGB new_point;
          new_point.x = pt.x+0.005*normal[0]*i;
          new_point.y = pt.y+0.005*normal[1]*i;
          new_point.z = pt.z+0.005*normal[2]*i;
          new_point.rgb = green.float_value;
          cloud->push_back(new_point);
        }
      }
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;

  publisher_.publish(*cloud);

  delete cloud;
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
      
      IcpLocal* tmpAlgo = new IcpLocal(cloud1_,cloud2_);

      if(algorithm_)
      {
        //tmpAlgo->SetTransformation(algorithm_->GetTransformation());
        delete algorithm_;
      }
      
      algorithm_ = tmpAlgo;
      
      algorithm_->SetMaxIterations(20);
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
    PCloud cloud(*cloud1_);
    lastTransformation_ *= algorithm_->GetTransformation();
    BOOST_FOREACH (pcl::PointXYZRGB& pt, cloud.points) {
      Eigen::Vector4f pnt(pt.x,pt.y,pt.z,1.0);
      pnt = lastTransformation_ * pnt;
      pt.x = pnt[0];
      pt.y = pnt[1];
      pt.z = pnt[2];    
    }
    
    *outCloud_ += cloud;
    
  }

  publisher_.publish(*outCloud_);
  
}
