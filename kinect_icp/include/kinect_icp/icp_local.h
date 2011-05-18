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

struct MatchedPoint
{
  Eigen::Vector3f first_point;
  Eigen::Vector3f second_point;
  Eigen::Vector3f normal;
  int x;
  int y;
  float distance;
  bool rejected;
};

class IcpLocal
{
public:
  IcpLocal(PCloud* first, PCloud* second, int iterations = 1);
  void Compute();

  void SetTransformation(const Eigen::Matrix4f& mat)
  {
    transformation_ = mat;
  };

  const Eigen::Matrix4f& GetTransformation() const
  {
    return transformation_;
  };

  float GetChange() const
  {
    return change_;
  }

  void SetMaxIterations(int iter)
  {
    maxIterations_ = iter;
  }
  
  void SetSelectionAmount(int amount)
  {
    selectionAmount_ = amount;
  }

  bool ComputeNormal(int x, int y, Eigen::Vector3f& normal);
  bool ComputeNormalSimple(int x, int y, Eigen::Vector3f& normal);

  //Testing
  void TestMinimizeTranslate();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
private:
  PCloud* first_;
  PCloud* second_;
  int maxIterations_;
  std::vector< MatchedPoint > selected_;

  int selectedCount_;
  float average_;
  float change_;

  Eigen::Matrix4f transformation_;

  void SelectMatchReject();
  void Selection();
  void Matching();
  void Rejecting();
  float Minimization();
  
  int selectionAmount_;

};

}

#endif
