#include <CL/cl.h>
#include "ros/ros.h"

class Vrip
{
public:

  Vrip();
  
  void fuseCloud(const PCloud::ConstPtr& new_point_cloud);

};
