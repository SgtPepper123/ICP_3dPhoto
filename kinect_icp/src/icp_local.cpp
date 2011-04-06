
#include "kinect_icp/icp_local.h"
#include "time.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace kinect_icp;
using namespace std;

IcpLocal::IcpLocal(PCloud* first, PCloud* second)
: first_(first)
, second_(second)
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

		
/*SomeMatrixClass IcpLocal::GetTransformation()
{
	return asdkasdkljas;
}
*/  
		
#define SelectionAmount 100
		
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
    mp.first_index = rand()%count;
    float x = first_->points[mp.first_index].x;
    if(x==x){    
      selected_.push_back(mp);      
      i++;
    }
  }
}

void IcpLocal::Matching()
{
  ROS_INFO("IcpLocal::Matching");
  int imax = selected_.size();
  for (int i=0; i<imax; i++)
  {
    selected_[i].distance = numeric_limits<float>::max();

    const Point& FirstPoint = first_->points[selected_[i].first_index];
    
    int jmax = second_->height;
    int kmax = second_->width;
    for (int j=1; j<jmax-1; j++)
    {
      for (int k=1; k<kmax-1; k++)
      {
        const Point& SecondPoint = (*second_)(j,k);
        if(!pcl::hasValidXYZ((*second_)(j,k)))
        {
          k++; //next will anyway not generate valid normal
          continue;
        }
        
        float dist = pcl::squaredEuclideanDistance(SecondPoint,FirstPoint);
        
        Eigen::Vector3f normal;

        if (dist < selected_[i].distance && ComputeNormal(j,k,normal)) 
        {
          selected_[i].distance = dist;
          selected_[i].second_index = j*kmax + k;
          selected_[i].normal = normal;
        }          
      }
    }
  }
}

bool IcpLocal::ComputeNormal(int j, int k, Eigen::Vector3f& normal)
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
  Eigen::Vector3f e0(South.x - North.x,South.y - North.y,South.z - North.z);
  Eigen::Vector3f e1(East.x - West.x,East.y - West.y,East.z - West.z);
  normal = e0.cross(e1);
  normal.normalize();
  return true;
}


void IcpLocal::Rejecting()
{
  ROS_INFO("IcpLocal::Rejecting");
  const double threshold = 10;

  int threshold_squared = threshold*threshold;
  int imax = selected_.size();
  for (int i=0; i<imax; i++)
  {
    selected_[i].rejected = selected_[i].distance > threshold_squared;
    /*if(selected_[i].rejected)
    {
       float dist_x = first_->points[j].x - second_->points[selected_[i].first_index].x;
      float dist_y = first_->points[j].y - second_->points[selected_[i].first_index].y;
      float dist_z = first_->points[j].z - second_->points[selected_[i].first_index].z;

     printf("%f, ", selected_[i].);
    }*/
    //printf("%f, ", selected_[i].distance);
    std::cout << "normal: " << selected_[i].normal << std::endl;
  }
}

float IcpLocal::Minimization()
{
  int N = selected_.size();
  
  Matrix<double, Dynamic, 6> A(N, 6);
  Matrix<double, Dynamic, 1> b(N, 1);

  for (int n = 0; n < N; n++) {
    // Fill in A
    Eigen::Vector3f normal = selected_[n].normal;
    Point source = first_->points[selected_[n].first_index];

    A(n, 0) = normal(2)*source.y - normal(1)*source.z;
    A(n, 1) = normal(0)*source.z - normal(2)*source.x;
    A(n, 2) = normal(1)*source.x - normal(0)*source.y;

    A(n, 3) = normal(0);
    A(n, 4) = normal(1);
    A(n, 5) = normal(2);

    // Fill in b
    Point dest = second_->points[selected_[n].second_index];
    b(n) = normal(0)*(dest.x-source.x) +
           normal(1)*(dest.y-source.y) +
           normal(2)*(dest.z-source.z);
  }

  // Least squares solve
  cout << "Result: ";
  cout << A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b) << endl;
  return 0.f;
}
