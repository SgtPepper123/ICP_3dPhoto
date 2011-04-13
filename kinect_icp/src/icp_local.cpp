
#include "kinect_icp/icp_local.h"
#include "time.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace kinect_icp;
using namespace std;

#define SelectionAmount 100
		
IcpLocal::IcpLocal(PCloud* first, PCloud* second)
: first_(first)
, second_(second)
, selectedCount_(0)
, transformation_(Matrix4f::Identity())
{
}
	

void IcpLocal::Compute(/*SomeMatrixClass initialTransformation*/)
{
  ROS_INFO("IcpLocal::Compute");
  float error = std::numeric_limits<float>::max();
  float old_error;
//  do
  {
    old_error = error;
    Selection();
    Matching();
    Rejecting();
    error = Minimization();
  }// while (abs(old_error-error) > ITERATION_THRESHOLD);
}
		
void IcpLocal::Selection()
{
  ROS_INFO("IcpLocal::Selection");
  srand(time(NULL));
  int i = 0;
  int count = first_->points.size();
  selected_.clear();
  selected_.reserve(SelectionAmount);
  MatchedPoint mp;
  while(i < SelectionAmount)
  {
    int index = rand()%count;
    const Point& tmp = first_->points[index];
    if(pcl::hasValidXYZ(tmp)){
      Vector4f pnt(tmp.x,tmp.y,tmp.z,1.0);
      pnt = transformation_ * pnt;    
      mp.first_point = Vector3f(pnt.x(),pnt.y(),pnt.z()); 
      selected_.push_back(mp);      
      i++;
    }
  }
}

void IcpLocal::Matching()
{
  ROS_INFO("IcpLocal::Matching");
  int imax = selected_.size();
  average_ = 0;
  for (int i=0; i<imax; i++)
  {
    selected_[i].distance = numeric_limits<float>::max();

    Vector3f FirstPoint = selected_[i].first_point;
    
    int xmax = second_->width;
    int ymax = second_->height;
    for (int y=1; y<ymax-1; y++)
    {
      for (int x=1; x<xmax-1; x++)
      {
        const Point& SecondPoint = (*second_)(x,y);
        if(!pcl::hasValidXYZ((*second_)(x,y)))
        {
          y++; //next will anyway not generate valid normal
          continue;
        }
        
        Vector3f SecondPnt(SecondPoint.x,SecondPoint.y,SecondPoint.z);
        Vector3f Dist = FirstPoint-SecondPnt;
        float dist = Dist.squaredNorm();
        
        Vector3f normal;

        if (dist < selected_[i].distance && ComputeNormal(x,y,normal)) 
        {
          selected_[i].distance = dist;
          selected_[i].second_point = SecondPnt;
          selected_[i].normal = normal;
        }          
      }
    }
    average_ += selected_[i].distance;
  }
  average_ /= (float)selected_.size();
}

bool IcpLocal::ComputeNormal(int j, int k, Vector3f& normal)
{
  const Point& North = (*second_)(j-1,k);
  if(!pcl::hasValidXYZ(North))
    return false;
  const Point& South = (*second_)(j+1,k);
  if(!pcl::hasValidXYZ(South))
    return false;
  const Point& West = (*second_)(j,k-1);
  if(!pcl::hasValidXYZ(West))
    return false;
  const Point& East = (*second_)(j,k+1);
  if(!pcl::hasValidXYZ(East))
    return false;
  Vector3f e0(South.x - North.x,South.y - North.y,South.z - North.z);
  Vector3f e1(East.x - West.x,East.y - West.y,East.z - West.z);
  normal = e0.cross(e1);
  normal.normalize();
  return true;
}


void IcpLocal::Rejecting()
{
  ROS_INFO("IcpLocal::Rejecting");
  const float threshold = 1.5;

  int imax = selected_.size();
  selectedCount_ = imax;
  for (int i=0; i<imax; i++)
  {
    selected_[i].rejected = selected_[i].distance > average_*threshold;
    if(selected_[i].rejected)
    {
      --selectedCount_;
    }
  }
  cout << "Rejected percentage: " << (1.f -(float)selectedCount_/(float)imax)*100.f << "%%" <<endl;
}

float IcpLocal::Minimization()
{
  ROS_INFO("IcpLocal::Minimization");
  int N = selected_.size();
  
  Matrix<double, Dynamic, Dynamic> A(selectedCount_, 6);
  Matrix<double, Dynamic, Dynamic> b(selectedCount_, 1);

  int i = 0;
  for (int n = 0; n < N; n++) {
    if(selected_[n].rejected)
    {
      continue;
    }
    // Fill in A
    Vector3f normal = selected_[n].normal;
    Vector3f& source = selected_[n].first_point;

    A(i, 0) = normal(2)*source(1) - normal(1)*source(2);
    A(i, 1) = normal(0)*source(2) - normal(2)*source(0);
    A(i, 2) = normal(1)*source(0) - normal(0)*source(1);

    A(i, 3) = normal(0);
    A(i, 4) = normal(1);
    A(i, 5) = normal(2);

    // Fill in b
    Vector3f dest_source = selected_[n].second_point - source;
    b(i) = normal.dot(dest_source);
    
    ++i;
  }

  // Least squares solve
  cout << "Result: ";
  cout << A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b) << endl;
  return 0.f;
}
