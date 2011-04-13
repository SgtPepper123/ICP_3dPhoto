
#include "kinect_icp/icp_local.h"
#include "time.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace kinect_icp;
using namespace std;

#define SelectionAmount 1000

#define RED "\033[31m\033[1m\033[5m"
#define GREEN "\033[32m\033[1m\033[5m"
#define YELLOW "\033[33m\033[1m\033[5m"
#define BLUE "\033[34m\033[1m\033[5m"
#define WHITE "\E[m"
		
IcpLocal::IcpLocal(PCloud* first, PCloud* second)
: first_(first)
, second_(second)
, selectedCount_(0)
, transformation_(Matrix4f::Identity())
{
  srand(time(NULL));
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
  //ROS_INFO("IcpLocal::Selection");
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
  //ROS_INFO("IcpLocal::Matching");
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
  //ROS_INFO("IcpLocal::Rejecting");
  const float threshold = 0.8;

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
  //ROS_INFO("IcpLocal::Minimization");
  int N = selected_.size();
  
  Matrix<double, Dynamic, Dynamic> A(selectedCount_, 6);
  Matrix<double, Dynamic, Dynamic> b(selectedCount_, 1);

  Matrix4f Transformation = Matrix4f::Identity();
  for(int iteration = 0; iteration < 5; ++iteration)
  {
    cout << "iteration " << iteration << ":" << endl;
    int i = 0;
    for (int n = 0; n < N; n++) {
      if(selected_[n].rejected)
      {
        continue;
      }
      // Fill in A
      Vector3f normal = selected_[n].normal;
      Vector3f& source = selected_[n].first_point;
      
      Vector4f src = transformation_*Vector4f(source(0),
                                              source(1),
                                              source(2),1);
      
      //selected_[n].first_point = Vector3f(src(0),src(1),src(2));

      A(i, 0) = normal(2)*src(1) - normal(1)*src(2);
      A(i, 1) = normal(0)*src(2) - normal(2)*src(0);
      A(i, 2) = normal(1)*src(0) - normal(0)*src(1);

      A(i, 3) = normal(0);
      A(i, 4) = normal(1);
      A(i, 5) = normal(2);

      // Fill in b 
      Vector3f dest_source = selected_[n].second_point - Vector3f(src(0),src(1),src(2));
      b(i) = normal.dot(dest_source);
      
      ++i;
    }

    // Least squares solve
    
    Matrix<double, Dynamic, Dynamic> TransformParams(6, 1);  
    TransformParams = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
    
    Transformation.topLeftCorner(3,3) = (
        AngleAxisf(TransformParams(0), Vector3f::UnitX()) *
        AngleAxisf(TransformParams(1), Vector3f::UnitY()) *
        AngleAxisf(TransformParams(2), Vector3f::UnitZ())
        ).toRotationMatrix();

    Transformation.topRightCorner(3,1) = Vector3f(TransformParams(3),
                                                  TransformParams(4),
                                                  TransformParams(5));
    
    cout << GREEN << "Params" << endl;
    cout << TransformParams << WHITE << endl;

    cout << RED << "lastIter" << endl;
    cout <<  Transformation << WHITE << endl;  
  
    transformation_ = Transformation * transformation_;

    cout << BLUE << "AllIter" << endl;
    Matrix3f tmpMat = transformation_.topLeftCorner(3,3);
    cout << tmpMat.eulerAngles(0,1,2) << endl;
    cout << transformation_ << WHITE << endl;  
    
    change_ = TransformParams.norm(); 
  }
  
  /*float error = 0;
  for (int n = 0; n < N; n++) {
    if(selected_[n].rejected)
    {
      continue;
    }
    // Fill in A
    Vector3f& source = selected_[n].first_point;
    Vector4f tmp(source.x(),source.y(),source.z(),1);
    
    tmp = Transformation * tmp;

    // Fill in b
    Vector3f dest_source = selected_[n].second_point - Vector3f(tmp.x(),tmp.y(),tmp.z());
    error += dest_source.squaredNorm();
  }
  
  error/=selectedCount_;
  
  cout << "average before" << average_ << " after " << error << endl;*/

  cout << "Result: ";
  cout << change_ << endl;
  return change_;
}


void IcpLocal::TestMinimizeTranslate()
{
  Eigen::Matrix3f rot = (AngleAxisf(0.2f, Vector3f::UnitX()) *
                         AngleAxisf(0.1f, Vector3f::UnitY()) *
                         AngleAxisf(0.0f, Vector3f::UnitZ())
                        ).toRotationMatrix();
  cout << rot << endl;
  selected_.clear();
  selected_.reserve(SelectionAmount);
  MatchedPoint mp;
  mp.normal = Eigen::Vector3f(0.f,0.f,1.f);
  mp.distance = 0.01f;
  mp.rejected = false;
  for(int i = 0; i<10; ++i)
  {
    for(int j = 0; j<10; ++j)
    {    
      mp.second_point = Eigen::Vector3f(1.0f*i,1.0f*j,0.0);
      mp.first_point = rot*mp.second_point;
      //mp.first_point[0] += 0.1f;
      //mp.first_point[1] += 0.2f;
      //mp.first_point[2] += 0.3f;
      selected_.push_back(mp);      
    }
  }
  mp.normal = Eigen::Vector3f(1.f,0.f,0.f);
  mp.distance = 0.01f;
  mp.rejected = false;
  for(int i = 0; i<10; ++i)
  {
    for(int j = 0; j<10; ++j)
    {    
      mp.second_point = Eigen::Vector3f(0.f,1.0f*j,1.f*i);
      mp.first_point = rot*mp.second_point;
      //mp.first_point[0] += 0.1f;
      //mp.first_point[1] += 0.2f;
      //mp.first_point[2] += 0.3f;
      selected_.push_back(mp);      
    }
  }
  mp.normal = Eigen::Vector3f(0.f,1.f,0.f);
  mp.distance = 0.01f;
  mp.rejected = false;
  for(int i = 0; i<10; ++i)
  {
    for(int j = 0; j<10; ++j)
    {    
      mp.second_point = Eigen::Vector3f(1.0f*j,0.f,1.f*i);
      mp.first_point = rot*mp.second_point;
      //mp.first_point[0] += 0.1f;
      //mp.first_point[1] += 0.2f;
      //mp.first_point[2] += 0.3f;
      selected_.push_back(mp);      
    }
  }
  selectedCount_ = selected_.size();
  Minimization();
}

