#include <CL/cl.h>
#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace kinect_icp
{

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCloud;

class Vrip
{
public:

  Vrip();
  ~Vrip();
  
  void fuseCloud(const PCloud::ConstPtr& new_point_cloud);
  
  class Vertex
  {
  public:
    float x,y,z;
  };
  
private:
  cl_command_queue command_queue_;
  cl_kernel kernel_;
  cl_kernel kernelMarching_;
  cl_program program_;
  cl_mem volume_mem_obj_;
  cl_mem march_mem_obj_;
  cl_mem image_mem_obj_;
  cl_context context_;

};

}
