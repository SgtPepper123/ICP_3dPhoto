#ifndef ICP_LOCAL
#define ICP_LOCAL

//#include <boost/unordered_set.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "ros/ros.h"
#include <google/dense_hash_set>

#include <tr1/unordered_map>

#define ITERATION_THRESHOLD 0.01

namespace kinect_icp
{

struct eqstr
{
  bool operator()(uint64_t s1, uint64_t s2) const
  {
    return (s1 == s2);
  }
};

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCloud;
typedef google::dense_hash_set<uint64_t, std::tr1::hash<uint64_t>, eqstr> Set;


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
  double Compute();

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
  int points1_;
  int points2_;
  float average_;
  float change_;

  Eigen::Matrix4f transformation_;
  Eigen::Matrix4f bestTransformation_;

  void SelectMatchReject();
  void Selection();
  void Matching();
  void Rejecting();
  float Minimization();
  double CalculateOverlap();
  int AddToHash(Set *hash, PCloud* cloud, bool transform);

  int selectionAmount_;
  int hashResolution_;
  double maxOverlap_;
  int MatchRadius_;
};

}

#endif
