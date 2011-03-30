#ifndef ICP_LOCAL
#define ICP_LOCAL

#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#define ITERATION_THRESHOLD 0.01

namespace kinect_icp
{

typedef pcl::PointCloud<pcl::PointXYZRGB> PCloud;

class IcpLocal
{
public:
  IcpLocal(const PCloud::ConstPtr& first, const PCloud::ConstPtr& second);
  void Compute(/*SomeMatrixClass initialTransformation*/);

  //SomeMatrixClass GetTransformation();  

private:
  PCloud::ConstPtr m_First;
  PCloud::ConstPtr m_Second;

  //SomeMatrixClass m_RelativeTransformation;  

  void Selection();
  void Matching();
  void Rejecting();
  float Minimization();
};

}

#endif
