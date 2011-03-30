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

private:

  //Members
  std::vector<PCloud::ConstPtr> Clouds_;

  PCloud* cloud1_;
  PCloud* cloud2_;

  ros::Publisher publisher_;
};

}

#endif
