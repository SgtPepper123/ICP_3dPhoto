#ifndef ICP_CORE
#define ICP_CORE

#include "kinect_icp/icp_local.h"
#include <boost/unordered_set.hpp>
#include <vector>

namespace kinect_icp
{

class IcpCore
{
public:
  IcpCore(ros::Publisher publisher);

  void registerCloud(const PCloud::ConstPtr& new_point_cloud);
  void registerHashCloud(const PCloud::ConstPtr& new_point_cloud);
  void visualizeNormals(const PCloud::ConstPtr& new_point_cloud);
  void generateGroundTruth(const PCloud::ConstPtr& new_point_cloud);
  void tuneParameters(const PCloud::ConstPtr& new_point_cloud);

  bool singleMerge_;
  bool accumulateResults_;

private:
  typedef boost::unordered_set<uint32_t> unordered_set;
  unordered_set point_hash;

  void oneIcpStep(const PCloud::ConstPtr& new_point_cloud);
  void publishDiffToStart();
  void compute(PCloud* cloud1, PCloud* cloud2, int max_iterations, int selection_amount);
  void transformCloud(PCloud* cloud, float color, bool transformCoordinates = true);
  Eigen::Matrix4f invertedTransformation(Eigen::Matrix4f original_matrix);
  void hashedAdd(PCloud* from, PCloud* to);

  //Members
  std::vector<PCloud::ConstPtr> Clouds_;

  ros::Publisher publisher_;

  PCloud* outCloud_;
  PCloud* cloud1_;
  PCloud* cloud2_;
  PCloud* firstCloud_;

  IcpLocal* algorithm_;

  double totalTime_;
  int numComputes_;

  float red_;
  float green_;
  float blue_;

  int frameNum_;

  Eigen::Matrix4f lastTransformation_;
};

}

#endif
