#ifndef ICP_LOCAL
#define ICP_LOCAL

#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#define ITERATION_THRESHOLD 0.01

namespace kinect_icp
{
typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCloud;

struct MatchedPoint {
  Eigen::Vector3f first_point;
  Eigen::Vector3f second_point;
  Eigen::Vector3f normal;
  float distance;
  bool rejected;
};

class IcpLocal
{
public:
  IcpLocal(PCloud* first, PCloud* second);
  void Compute();

  const Eigen::Matrix4f&  GetTransformation() const { return transformation_; };
  float                   GetChange()         const { return change_; } 

  //Testing
  void TestMinimizeTranslate();

private:
  PCloud* first_;
  PCloud* second_;
  
  std::vector< MatchedPoint > selected_;
  int selectedCount_;
  float average_;
  float change_;
  
  Eigen::Matrix4f transformation_;

  void Selection();
  void Matching();
  void Rejecting();
  float Minimization();  
  
  bool ComputeNormal(int j, int k, Eigen::Vector3f& normal);
};

}

#endif
