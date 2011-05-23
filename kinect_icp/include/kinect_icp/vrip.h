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

  void marchingCubes();
  int preFixSum(cl_mem input, cl_mem output, int BlockSize);
  void loadKernel(const char* filename, int num_kernels, cl_kernel* kernels[],
  const char* kernel_names[]);

  class Vertex
  {
  public:
    float x, y, z;
  };

private:
  cl_command_queue command_queue_;
  cl_kernel fuse_kernel_;
  cl_kernel preMarching_;
  cl_kernel mainMarching_;
  cl_device_id device_id_;
  cl_mem volume_mem_obj_;
  cl_mem march_mem_obj_;
  cl_mem image_mem_obj_;
  cl_context context_;

  int imageSize_;
  int volumeSize_;

};

}
