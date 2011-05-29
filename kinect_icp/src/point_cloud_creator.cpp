#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <boost/bind.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>


typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<pcl::PointXYZRGB> PCloud;

using namespace Eigen;

typedef union
{

  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;

struct Plane
{
  Eigen::Vector3f dir;
  float alpha;
};

float planedist(const Plane& plane, const Eigen::Vector3f dir, const Eigen::Vector3f pos)
{
  float denom = plane.dir.dot(dir);
  if(abs(denom) > 0.0001)
    return 10.f;
  return -(plane.dir.dot(pos) + plane.alpha)/denom;
}

int main(int argc, char **argv)
{

  //intialize some ros stuff
  ros::init(argc, argv, "point_cloud_creator");

  //create ros node to be able to send/receive messages
  ros::NodeHandle n;

  // Create a Publisher for the final, merged images
  ros::Publisher publisher = n.advertise< PCloud const >("/camera/rgb/points", 10);
  
  ros::Rate loop_rate(.5);
  
  int count = 0;
  while (ros::ok())
  {
    loop_rate.sleep();
    
    PCloud cloud;
    
    cloud.width = 640;
    cloud.height = 481;
    
    cloud.points.resize(640*481);
    
    ros::Time time = ros::Time::now () + ros::Duration(1.0);

    cloud.header.stamp = time;
    cloud.header.frame_id = "/openni_rgb_optical_frame";
    cloud.header.seq = 10 + count;
    
    RGBValue color;
    //color.float_value = pt.rgb;
    color.Red = 0;
    color.Green = 0;
    color.Blue = 255;

    Plane plane1;
    plane1.dir << 0.0, 0.0, -1.0;
    plane1.alpha = 3.0;
    Plane plane2;
    plane2.dir << 0.0, -1.0, -1.0;
    plane2.dir.normalize();
    plane2.alpha = 2.0;
    
    for(int x = 0; x < cloud.width; ++x)
    {
      for(int y = 0; y < cloud.height; ++y)
      {
        Eigen::Vector3f vec;
        vec << (float)x*0.001904761904762 - 0.608571428571429,
               (float)y*0.001904761904762 - 0.456190476190476,
               1.f;//sqrt(dx*dx+dy*dy+3.0*3.0);
        vec.normalize();
        Eigen::Vector3f pos;
        pos << (float)count*0.1f - 1.0f, (float)count*0.1f - 1.0f, -(float)count*0.0f;
        
        cloud.sensor_origin_(0) = pos(0); 
        cloud.sensor_origin_(1) = pos(1);
        cloud.sensor_origin_(2) = pos(2);

        //std::cout << planedist(plane, vec, pos) << std::endl;
        vec *= std::min(planedist(plane1, vec, pos),planedist(plane2, vec, pos));

        /*Matrix<float, 3, 3 > P;
        P << 525.0, 0, 319.5,
          0, 525.0, 239.5,
          0, 0, 1;*/
          
        //std::cout << vec << std::endl;

        Point p;
        p.x = vec(0);
        p.y = vec(1);
        p.z = vec(2);
        p.rgb = color.float_value;
        cloud(x,y) = p;
      }
    }
    
    Point p1;
    p1.x = cloud.sensor_origin_(0);
    p1.y = cloud.sensor_origin_(1);
    p1.z = cloud.sensor_origin_(2);
    p1.rgb = 1.f;
    
    cloud(0,480) = p1;

    std::cout << cloud << std::endl;
    publisher.publish(cloud);
    std::cout << "test" << std::endl;

    ros::spinOnce();

    ++count;
    std::cout << count << std::endl;
  }

  //entrypoint of the Icp Algorithm
  //kinect_icp::Vrip vrip;

  //create function pointer to registerCloud function in IcpCore
  //then subscribe it with the rosout node, so all published pointclouds will be directed to the algorithm.
  //boost::function< void(const boost::shared_ptr< kinect_icp::PCloud const > &) > func = boost::bind(&kinect_icp::Vrip::fuseCloud, boost::ref(vrip), _1);
  //ros::Subscriber sub = n.subscribe("/camera/rgb/points", 1, func);

  //wait for messages till canceled
  //ros::spin();

  return 0;
}
