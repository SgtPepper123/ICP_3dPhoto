#ifndef ICP_LOCAL
#define ICP_LOCAL

#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#define ITERATION_THRESHOLD 0.01

namespace kinect_icp
{

typedef pcl::PointCloud<pcl::PointXYZRGB> PCloud;

struct MatchedPoint {
  int first_index;
  int second_index;
  double distance;
};

class IcpLocal
{
public:
  IcpLocal(PCloud* first, PCloud* second);
  void Compute(/*SomeMatrixClass initialTransformation*/);

  //SomeMatrixClass GetTransformation();  

private:
  PCloud* first_;
  PCloud* second_;
  
  std::vector< MatchedPoint > selected_;

  //SomeMatrixClass m_RelativeTransformation;  

  void Selection();
  void Matching();
  void Rejecting();
  float Minimization();
};

}

#endif
