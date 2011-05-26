#include "kinect_icp/icp_core.h"
#include <Eigen/Dense>

using namespace kinect_icp;
using namespace Eigen;
using namespace std;

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

IcpCore::IcpCore(ros::Publisher publisher)
  : singleMerge_(false)
  , accumulateResults_(true)
  , publisher_(publisher)
  , outCloud_(NULL)
  , cloud1_(NULL)
  , cloud2_(NULL)
  , algorithm_(NULL)
  , totalTime_(0)
  , numComputes_(0)
  , frameNum_(0)
  , lastTransformation_(Eigen::Matrix4f::Identity())
{
  Clouds_.reserve(1000);

  RGBValue green;
  green.Red = 0;
  green.Green = 255;
  green.Blue = 0;

  green_ = green.float_value;

  RGBValue red;
  red.Red = 255;
  red.Green = 0;
  red.Blue = 0;

  red_ = red.float_value;

  RGBValue blue;
  blue.Red = 0;
  blue.Green = 0;
  blue.Blue = 255;

  blue_ = blue.float_value;
}

void IcpCore::visualizeNormals(const PCloud::ConstPtr& new_point_cloud)
{
  PCloud* cloud = new PCloud(*new_point_cloud);

  int width = cloud->width;
  int height = cloud->height;

  IcpLocal algorithm(cloud, cloud);

  srand(42);

  for (int i = 0; i < width; i++)
  {
    // Select random point
    int x = (rand() % (width - 2 * 10)) + 10;
    int y = (rand() % (height - 2 * 10)) + 10;
    Point& pt = (*cloud)(x, y);
    if (pcl::hasValidXYZ(pt))
    {
      pt.rgb = red_;

      for (int i = 0; i < 20; i++)
      {
        Vector3f normal;
        if (algorithm.ComputeNormalSimple(x, y, normal))
        {
          if (normal[1] > 0)
            normal = -normal;
          pcl::PointXYZRGB new_point;
          new_point.x = pt.x + 0.005 * normal[0] * i;
          new_point.y = pt.y + 0.005 * normal[1] * i;
          new_point.z = pt.z + 0.005 * normal[2] * i;
          new_point.rgb = green_;
          cloud->push_back(new_point);
        }
      }
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;

  publisher_.publish(*cloud);

  delete cloud;
}

void IcpCore::oneIcpStep(const PCloud::ConstPtr& new_point_cloud)
{
  if (cloud2_)
  {
    delete cloud2_;
    cloud2_ = NULL;
  }

  cloud2_ = cloud1_;
  cloud1_ = new PCloud(*new_point_cloud);

  if (algorithm_)
  {
    delete algorithm_;
  }

  compute(cloud1_, cloud2_, 100, 100);
}

void IcpCore::compute(PCloud* cloud1, PCloud* cloud2, int max_iterations, int selection_amount)
{
  algorithm_ = new IcpLocal(cloud1, cloud2);
  algorithm_->SetMaxIterations(max_iterations);
  algorithm_->SetSelectionAmount(selection_amount);
  cout << "Time: " << algorithm_->Compute() << endl;

  lastTransformation_ *= algorithm_->GetTransformation();
}

void IcpCore::publishDiffToStart()
{

  PCloud out(*firstCloud_);
  PCloud cloud(*cloud1_);
  transformCloud(&cloud, red_);

  out += cloud;
  publisher_.publish(out);
}

void IcpCore::transformCloud(PCloud* cloud, float color, bool transformCoordinates)
{

  BOOST_FOREACH(pcl::PointXYZRGB& pt, cloud->points)
  {
    Eigen::Vector4f pnt(pt.x, pt.y, pt.z, 1.0);

    pt.rgb = color;

    if (transformCoordinates)
    {
      pnt = lastTransformation_ * pnt;
      pt.x = pnt[0];
      pt.y = pnt[1];
      pt.z = pnt[2];
    }
  }
}

Eigen::Matrix4f IcpCore::invertedTransformation(Eigen::Matrix4f original_matrix)
{
  Matrix3f rotation = original_matrix.topLeftCorner(3, 3);
  Vector3f translation = original_matrix.topRightCorner(3, 1);
  Vector3f angles = rotation.eulerAngles(0, 1, 2);

  Matrix4f result = Eigen::Matrix4f::Identity();

  result.topLeftCorner(3, 3) = (
    AngleAxisf(-angles(0), Vector3f::UnitX()) *
    AngleAxisf(-angles(1), Vector3f::UnitY()) *
    AngleAxisf(-angles(2), Vector3f::UnitZ())
    ).toRotationMatrix();


  result.topRightCorner(3, 1) = -translation;

  return result;
}

void IcpCore::hashedAdd(PCloud* from, PCloud* to)
{
  Eigen::Matrix4f inverted = invertedTransformation(lastTransformation_);

  BOOST_FOREACH(pcl::PointXYZRGB& pt, from->points)
  {
    Eigen::Vector4f pnt(pt.x, pt.y, pt.z, 1.0);
    pnt = inverted * pnt;
    pt.x = pnt[0];
    pt.y = pnt[1];
    pt.z = pnt[2];

    uint8_t first = pt.x * 100;
    uint8_t second = pt.y * 100;
    uint8_t third = pt.z * 100;
    uint32_t hash_value = first + (second << 8) + (third << 16);
    if (pcl::hasValidXYZ(pt) && point_hash.find(hash_value) == point_hash.end())
    {
      to->push_back(pt);
      point_hash.insert(hash_value);
    }
  }

  to->width = outCloud_->points.size();

  ROS_DEBUG("Have %i hashed points...", to->width);
}

void IcpCore::registerHashCloud(const PCloud::ConstPtr& new_point_cloud)
{
  ROS_DEBUG("Received Point Cloud");

  if (!cloud1_)
  {
    cloud1_ = new PCloud(*new_point_cloud);
    outCloud_ = new PCloud();
    outCloud_->header = new_point_cloud->header;

    outCloud_->sensor_origin_ = new_point_cloud->sensor_origin_;
    outCloud_->sensor_orientation_ = new_point_cloud->sensor_orientation_;
    outCloud_->is_dense = true;
    outCloud_->height = 1;

    hashedAdd(cloud1_, outCloud_);

    publisher_.publish(*outCloud_);
    return;
  }

  cloud1_ = new PCloud(*new_point_cloud);

  if (algorithm_)
  {
    delete algorithm_;
  }
  algorithm_ = new IcpLocal(outCloud_, cloud1_);
  algorithm_->SetTransformation(lastTransformation_);
  algorithm_->SetMaxIterations(200);

  numComputes_++;
  totalTime_ += algorithm_->Compute();
  cout << "Average: " << totalTime_ / numComputes_ << " ms (" << totalTime_ << "/" << numComputes_ << ")" << endl;

  lastTransformation_ = algorithm_->GetTransformation();

  hashedAdd(cloud1_, outCloud_);

  publisher_.publish(*outCloud_);
}

void IcpCore::registerCloud(const PCloud::ConstPtr& new_point_cloud)
{
  ROS_DEBUG("Received Point Cloud");

  if (singleMerge_)
  {
    if (!algorithm_)
    {
      if (!cloud1_)
      {
        cloud1_ = new PCloud(*new_point_cloud);
        return;
      }

      cloud2_ = new PCloud(*new_point_cloud);

      algorithm_ = new IcpLocal(cloud1_, cloud2_);
    }

    if (outCloud_)
    {
      delete outCloud_;
      outCloud_ = NULL;
    }

    outCloud_ = new PCloud(*cloud1_);
  }
  else
  {
    if (!cloud1_)
    {
      cloud1_ = new PCloud(*new_point_cloud);
      outCloud_ = new PCloud(*new_point_cloud);
      publisher_.publish(new_point_cloud);
      return;
    }

    if (cloud2_)
    {
      delete cloud2_;
      cloud2_ = NULL;
    }

    cloud2_ = cloud1_;
    cloud1_ = new PCloud(*new_point_cloud);
    if (!accumulateResults_)
    {
      delete outCloud_;
      outCloud_ = new PCloud(*cloud2_);
    }
    IcpLocal* tmpAlgo = new IcpLocal(cloud1_, cloud2_);

    if (algorithm_)
    {
      //tmpAlgo->SetTransformation(algorithm_->GetTransformation());
      delete algorithm_;
    }

    algorithm_ = tmpAlgo;

    algorithm_->SetMaxIterations(200);

  }

  //algorithm.TestMinimizeTranslate();
  numComputes_++;
  totalTime_ += algorithm_->Compute();
  cout << "Average: " << totalTime_ / numComputes_ << " ms (" << totalTime_ << "/" << numComputes_ << ")" << endl;

  if (singleMerge_)
  {
    RGBValue color;
    //color.float_value = pt.rgb;
    color.Red = 0;
    color.Green = 0;
    color.Blue = 255;

    Eigen::Matrix4f mat = algorithm_->GetTransformation();

    BOOST_FOREACH(pcl::PointXYZRGB& pt, outCloud_->points)
    {
      Eigen::Vector4f pnt(pt.x, pt.y, pt.z, 1.0);
      pnt = mat * pnt;
      pt.x = pnt[0];
      pt.y = pnt[1];
      pt.z = pnt[2];
      pt.rgb = color.float_value;
    }

    color.Blue = 0;
    color.Red = 255;

    BOOST_FOREACH(pcl::PointXYZRGB& pt, cloud2_->points)
    {
      pt.rgb = color.float_value;
    }

    *outCloud_ += *cloud2_;
    algorithm_->SetMaxIterations(1);
  }
  else
  {
    PCloud cloud(*cloud1_);
    if (accumulateResults_)
    {
      lastTransformation_ *= algorithm_->GetTransformation();
    }
    else
    {
      lastTransformation_ = algorithm_->GetTransformation();
    }

    BOOST_FOREACH(pcl::PointXYZRGB& pt, cloud.points)
    {
      Eigen::Vector4f pnt(pt.x, pt.y, pt.z, 1.0);
      pnt = lastTransformation_ * pnt;
      pt.x = pnt[0];
      pt.y = pnt[1];
      pt.z = pnt[2];
    }

    *outCloud_ += cloud;

  }

  publisher_.publish(*outCloud_);
}

void IcpCore::tuneParameters(const PCloud::ConstPtr& new_point_cloud)
{
  const int max_frame = 49;

  // First step
  if (!cloud1_)
  {
    frameNum_++;

    cloud1_ = new PCloud(*new_point_cloud);
    return;
  }

  // Incremental mode
  if (frameNum_ < max_frame)
  {
    frameNum_++;
    cout << "Frame: " << frameNum_ << endl;

    oneIcpStep(new_point_cloud);
  }

  if (frameNum_ == max_frame)
  {
    /*Final Angles:

Final Translation
     */

    PCloud bad(*cloud1_);
    transformCloud(&bad, red_);
    cout << "Bad transformation:" << endl;
    Matrix3f rotation = lastTransformation_.topLeftCorner(3, 3);
    Vector3f translation = lastTransformation_.topRightCorner(3, 1);
    Vector3f angles = rotation.eulerAngles(0, 1, 2);

    cout << "Angles:" << endl;
    cout << angles << endl;
    cout << "Translation:" << endl;
    cout << translation << endl;

    angles = Vector3f(-0.0507161, 0.0205142, 0.0105023);
    translation = Vector3f(0.552078, -0.11558, -0.0119312);

    lastTransformation_.topLeftCorner(3, 3) = (
      AngleAxisf(angles(0), Vector3f::UnitX()) *
      AngleAxisf(angles(1), Vector3f::UnitY()) *
      AngleAxisf(angles(2), Vector3f::UnitZ())
      ).toRotationMatrix();


    lastTransformation_.topRightCorner(3, 1) = translation;

    transformCloud(cloud1_, green_);
    cout << "Good transformation:" << endl;
    cout << "Angles:" << endl;
    cout << angles << endl;
    cout << "Translation:" << endl;
    cout << translation << endl;

    *cloud1_ += bad;

    publisher_.publish(*cloud1_);
  }
}

void IcpCore::generateGroundTruth(const PCloud::ConstPtr& new_point_cloud)
{
  ROS_DEBUG("Received Point Cloud");

  const int max_frame = 49;
  const int precision_diff = 10;
  const int precision_steps = 5;
  const int final_average_steps = 5;

  // First step
  if (!cloud1_)
  {
    cloud1_ = new PCloud(*new_point_cloud);
    firstCloud_ = new PCloud(*new_point_cloud);

    transformCloud(firstCloud_, green_);

    return;
  }

  // Incremental mode
  if (frameNum_ < max_frame)
  {
    frameNum_++;
    cout << "Frame: " << frameNum_ << endl;

    oneIcpStep(new_point_cloud);
    publishDiffToStart();
  }

  // Refinement mode
  if (frameNum_ % precision_diff == 0 || frameNum_ == max_frame)
  {
    for (int j = 0; j < precision_steps; j++)
    {
      PCloud cloud(*cloud1_);

      transformCloud(&cloud, red_);
      compute(&cloud, firstCloud_, 100, 1000);

      cout << "Refinement: " << j << endl;

      publishDiffToStart();
    }

    if (frameNum_ == max_frame)
    {
      Vector3f averageAngles(0, 0, 0);
      Vector3f averageTranslation(0, 0, 0);

      for (int j = 0; j < final_average_steps; j++)
      {
        PCloud cloud(*cloud1_);

        transformCloud(&cloud, red_);
        compute(&cloud, firstCloud_, 100, 1000);
        publishDiffToStart();

        Matrix3f rotation = lastTransformation_.topLeftCorner(3, 3);
        Vector3f translation = lastTransformation_.topRightCorner(3, 1);
        Vector3f angles = rotation.eulerAngles(0, 1, 2);
        averageAngles += angles;
        averageTranslation += translation;

        cout << "Final Refinement: " << j << endl;
        cout << angles << endl;
        cout << translation << endl;
      }

      averageAngles /= final_average_steps;
      averageTranslation /= final_average_steps;

      cout << "Final Angles: " << endl;
      cout << averageAngles << endl;
      cout << "Final Translation" << endl;
      cout << averageTranslation << endl;

      lastTransformation_.topLeftCorner(3, 3) = (
        AngleAxisf(averageAngles(0), Vector3f::UnitX()) *
        AngleAxisf(averageAngles(1), Vector3f::UnitY()) *
        AngleAxisf(averageAngles(2), Vector3f::UnitZ())
        ).toRotationMatrix();


      lastTransformation_.topRightCorner(3, 1) = averageTranslation;

      publishDiffToStart();
    }
  }
}