#ifndef ICP_CORE
#define ICP_CORE

#include "kinect_icp/icp_local.h"
#include <vector>

namespace kinect_icp
{

class IcpCore
{
public:
  IcpCore(ros::Publisher publisher);

  void registerCloud(const PCloud::ConstPtr& new_point_cloud);
  void visualizeNormals(const PCloud::ConstPtr& new_point_cloud);

  bool singleMerge_;
  bool accumulateResults_;

private:

  void oneIcpStep(const PCloud::ConstPtr& new_point_cloud);
  void publishDiffToStart();

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
