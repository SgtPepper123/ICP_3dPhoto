
#include "kinect_icp/icp_local.h"
#include "time.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace kinect_icp;
using namespace std;

#define SelectionAmount 60

#define RED "\033[31m\033[1m\033[5m"
#define GREEN "\033[32m\033[1m\033[5m"
#define YELLOW "\033[33m\033[1m\033[5m"
#define BLUE "\033[34m\033[1m\033[5m"
#define WHITE "\E[m"

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

//#define PrintMinimizationMatrices
		
IcpLocal::IcpLocal(PCloud* first, PCloud* second, int iterations)
: first_(first)
, second_(second)
, maxIterations_(iterations)
, selectedCount_(0)
, transformation_(Matrix4f::Identity())
{
  //srand(time(NULL));
  srand(42);
}
	
#define MinValidIterations 3

void IcpLocal::Compute(/*SomeMatrixClass initialTransformation*/)
{
  ROS_INFO("IcpLocal::Compute");
  float start = clock();
  float error = std::numeric_limits<float>::max();
  float old_error;
  int iterations = 1;
  int validIterations = 0;
  do
  {
    //cout << "IcpIteration " << iterations << ":" << endl;
    old_error = error;
    //Selection();
    //Matching();
    //Rejecting();
    SelectMatchReject();
    error = Minimization();
    iterations++; 
    if(GetChange()<0.01)
    {
      if(++validIterations == MinValidIterations)
      {
        break;
      }else
      {
        validIterations = 0;
      }
    }
  }while(iterations < maxIterations_);
  std::cout << "time: " << ((float)clock()-start)*1000.0/((float)CLOCKS_PER_SEC) << std::endl;
  ROS_INFO("IcpLocal::ComputeFinished");
}
		
const int MatchRadius = 2;
const int Radius = 2;

void IcpLocal::SelectMatchReject()
{
  //ROS_INFO("IcpLocal::SelectMatchReject");
  Matrix<float, 3, 4> P;
  P <<  525.0,      0,  319.5,  0,
            0,  525.0,  239.5,  0,
            0,      0,      1,  0;  

  int width = first_->width;    
  int height = first_->height;
  selected_.clear();
  selected_.reserve(SelectionAmount);
  
  float sum = 0.f;
  //average guessing
  for (int i = 0; i < 10;)
  {
    const Point& tmp = (*first_)(rand()%width,rand()%height);
    Vector4f p1(tmp.x, tmp.y, tmp.z, 1.f);
    
    p1 = transformation_ * p1;
    
    Vector3f coords = P * p1;
    
    //+0.5 is there to round to nearst int and not just floor
    int x = coords[0]/coords[2] + 0.5;
    int y = coords[1]/coords[2] + 0.5;
       
    int xmax = second_->width;
    int ymax = second_->height;

    if (x < 0 || y < 0 || x >= xmax || y >= ymax) {
      continue;
    }

    const Point& SecondPoint = (*second_)(x,y);
    if(!pcl::hasValidXYZ(SecondPoint))
    {
      continue;
    }
      
    Vector4f p2(SecondPoint.x,SecondPoint.y,SecondPoint.z,1.0);
    sum += (p1-p2).squaredNorm();
    ++i; 
  }
    
  for (int i = 0; i < SelectionAmount; )
  {
    MatchedPoint mp;
    int x1 = rand()%width;
    int y1 = rand()%height;
    const Point& tmp = (*first_)(x1,y1);
    if(pcl::hasValidXYZ(tmp)){
      mp.first_point = Vector3f(tmp.x,tmp.y,tmp.z);
      Vector4f p1(tmp.x, tmp.y, tmp.z, 1.f);
      
      p1 = transformation_ * p1;
      
      Vector3f coords = P * p1;
      
      //+0.5 is there to round to nearst int and not just floor
      int x = coords[0]/coords[2];
      int y = coords[1]/coords[2];
         
      int xmax = second_->width;
      int ymax = second_->height;

      if (x < MatchRadius || y < MatchRadius || x >= xmax-MatchRadius || y >= ymax-MatchRadius) {
        continue;
      }
      
      bool reject = true;
      
      int bestX = x;
      int bestY = y;
      Vector4f bestPoint;
      float bestColorDist = 0;
      float bestDist = 0;
      
      RGBValue color;
      color.float_value = tmp.rgb;
      Vector3f color1(color.Red, color.Green, color.Blue);
            
      for (int xx = x-MatchRadius; xx<=x+MatchRadius; ++xx)
      {
        for (int yy = y-MatchRadius; yy<=y+MatchRadius; ++yy)
        {
          const Point& SecondPoint = (*second_)(xx,yy);
          if(!pcl::hasValidXYZ(SecondPoint))
          {
            continue;
          }
          
          Vector4f p2(SecondPoint.x,SecondPoint.y,SecondPoint.z,1.0);
          float dist = (p1-p2).squaredNorm();
          
          if(dist > 1.2 * sum/((float)(10+i)))
          {
            continue;
          }
          
          color.float_value = SecondPoint.rgb;
          Vector3f color2(color.Red, color.Green, color.Blue);
          float colorDist = (color1 - color2).squaredNorm();
          
          if(reject)
          {
            reject = false;
          }
          else
          {
            if(colorDist > bestColorDist)
            {
              continue;
            }
          }
          bestX = xx;
          bestY = yy;
          bestColorDist = colorDist;
          bestDist = dist;                        
        }
      }
      
      if(reject)
      {
        continue;
      }
      

      Vector3f normal;
      //cout << RED << "NormalCoordinates" << selected_[i].x << ", " << selected_[i].y << WHITE << endl;
      if(!ComputeNormal(bestX,bestY,normal))
      {
        continue;
      }

      sum += bestDist;
        
      mp.normal = normal;
               
      const Point& MatchedPoint = (*second_)(bestX,bestY);
      mp.second_point = Vector3f(MatchedPoint.x, MatchedPoint.y, MatchedPoint.z);
      mp.rejected = false;
      selected_.push_back(mp);      
      i++;
    }
  }
  
  selectedCount_ = selected_.size();
}
		
void IcpLocal::Selection()
{
  ROS_INFO("IcpLocal::Selection");
  int i = 0;
  int width = first_->width;
  int height = first_->height;
  selected_.clear();
  selected_.reserve(SelectionAmount);
  MatchedPoint mp;
  while(i < SelectionAmount)
  {
    int x = rand()%width;
    int y = rand()%height;
    const Point& tmp = (*first_)(x,y);
    if(pcl::hasValidXYZ(tmp)){
      mp.first_point = Vector3f(tmp.x,tmp.y,tmp.z);
      selected_.push_back(mp);      
      i++;
    }
  }
}

// TODO optimize
void IcpLocal::Matching()
{
  ROS_INFO("IcpLocal::Matching");
  int imax = selected_.size();
  average_ = 0;
  int good_count = 0;
  Matrix<float, 3, 4> P;
  P <<  525.0,      0,  319.5,  0,
            0,  525.0,  239.5,  0,
            0,      0,      1,  0;  

  for (int i=0; i<imax; i++)
  {    
    Vector3f& tmp = selected_[i].first_point;
    Vector4f pnt(tmp[0],tmp[1],tmp[2],1.0);
    
    pnt = transformation_ * pnt;
    Vector3f FirstPoint = Vector3f(pnt[0],pnt[1],pnt[2]);
    
    Vector3f coords = P * pnt;
    
    //+0.5 is there to round to nearst int and not just floor
    int x = coords[0]/coords[2] + 0.5;
    int y = coords[1]/coords[2] + 0.5;
       
    int xmax = second_->width;
    int ymax = second_->height;

    if (x < 0 || y < 0 || x >= xmax || y >= ymax) {
      selected_[i].rejected = true;
      continue;
    }

    const Point& SecondPoint = (*second_)(x,y);
    if(!pcl::hasValidXYZ(SecondPoint))
    {
      selected_[i].rejected = true;
      continue;
    }
      
    selected_[i].rejected = false;
    
    Vector3f SecondPnt(SecondPoint.x,SecondPoint.y,SecondPoint.z);
    Vector3f Dist = FirstPoint-SecondPnt;
    float dist = Dist.squaredNorm();
        
    selected_[i].second_point = SecondPnt;
    selected_[i].x = x;
    selected_[i].y = y;

    selected_[i].distance = sqrt(dist);
    average_ += selected_[i].distance;
    good_count++;
  }

  average_ /= (float)good_count;

}

bool IcpLocal::ComputeNormalSimple(int x, int y, Vector3f& normal)
{
  const Point& point_1 = (*second_)(x+Radius, y);
  const Point& point_2 = (*second_)(x-Radius, y);
  const Point& point_3 = (*second_)(x, y+Radius);
  const Point& point_4 = (*second_)(x, y-Radius);

  if (!pcl::hasValidXYZ(point_1) || !pcl::hasValidXYZ(point_2) || !pcl::hasValidXYZ(point_3) || !pcl::hasValidXYZ(point_4))
    return false;

  Vector3f first(point_1.x-point_2.x, point_1.y-point_2.y, point_1.z-point_2.z);
  Vector3f second(point_3.x-point_4.x, point_3.y-point_4.y, point_3.z-point_4.z);
  
  normal = first.cross(second);
  normal.normalize();

  return true;
}

bool IcpLocal::ComputeNormal(int x, int y, Vector3f& normal)
{
  int diam = Radius*2+1;
  diam *= diam;
  Matrix<double, 3, Dynamic> A(3, diam);
  Vector3f average(0.0, 0.0, 0.0);
  
  int count = 0;
  for (int xdiff = -Radius; xdiff <= Radius; xdiff++) {
    for (int ydiff = -Radius; ydiff <= Radius; ydiff++) {
      const Point& current = (*second_)(x+xdiff, y+ydiff);

      // Check if point is valid
      if (!pcl::hasValidXYZ(current))
        continue;

      // Add point to average
      average += Vector3f(current.x, current.y, current.z);

      // Add point to matrix
      A(0, count) = current.x;
      A(1, count) = current.y;
      A(2, count) = current.z;

      count++;
    }
  }

  if (count < 4) {
    return false;
  }

  average /= count;

  for (int i=0; i<count; i++) {
    A(0, i) = A(0, i) - average(0);
    A(1, i) = A(1, i) - average(1);
    A(2, i) = A(2, i) - average(2);
  }

  A.resize(3, count);

  Matrix<double, 3, 3> cov = A*A.transpose();
  EigenSolver< Matrix<double, 3, 3> > es(cov);

  float ev0 = abs(es.eigenvalues()[0]);
  float ev1 = abs(es.eigenvalues()[1]);
  float ev2 = abs(es.eigenvalues()[2]);

  int i = 2;
  if (ev0 < ev1 && ev0 < ev2) 
    i = 0;
  else if (ev1 < ev0 && ev1 < ev2)
    i = 1;
//  else
//    normal = es.eigenvectors().col(2);

  normal(0) = real(es.eigenvectors()(0, i));
  normal(1) = real(es.eigenvectors()(1, i));
  normal(2) = real(es.eigenvectors()(2, i));

  if (normal.norm() > 1.1 || normal.norm() < 0.9 || normal.norm() != normal.norm()) {

    /*cout << "PROBLEM" << endl;
    cout << normal << endl;
    cout << "A:" << endl;
    cout << A << endl;
    cout << "EVS" << ev0 << "," << ev1 << "," << ev2 << endl;
    */
    return false;
  }

  return true;
}


void IcpLocal::Rejecting()
{
  ROS_INFO("IcpLocal::Rejecting");
  const float threshold = 1000;

  int imax = selected_.size();
  selectedCount_ = imax;
  for (int i=0; i<imax; i++)
  {
    if (!selected_[i].rejected)
      selected_[i].rejected = selected_[i].distance > average_*threshold;

    if(selected_[i].rejected)
    {
      --selectedCount_;
    }
    else
    {
        Vector3f normal;
        //cout << RED << "NormalCoordinates" << selected_[i].x << ", " << selected_[i].y << WHITE << endl;
        if(ComputeNormal(selected_[i].x,selected_[i].y,normal))
        {
           selected_[i].normal = normal;         
        }else
        {
          selected_[i].rejected = true;
          --selectedCount_;
        }
    }
  }
  cout << "Rejected percentage: " << (1.f -(float)selectedCount_/(float)imax)*100.f << "%" <<endl;
}

float IcpLocal::Minimization()
{
  //ROS_INFO("IcpLocal::Minimization");
  int N = selected_.size();
  
  Matrix<double, Dynamic, Dynamic> A(selectedCount_, 6);
  Matrix<double, Dynamic, Dynamic> b(selectedCount_, 1);

  Matrix4f Transformation = Matrix4f::Identity();
  change_ = 0;
  for(int iteration = 0; iteration < 1; ++iteration)
  {
#ifdef PrintMinimizationMatrices
    cout << "iteration " << iteration << ":" << endl;
#endif
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
    
#ifdef PrintMinimizationMatrices 
    /*cout << GREEN << "Params" << endl;
    cout << TransformParams << WHITE << endl;

    cout << RED << "lastIter" << endl;
    cout <<  Transformation << WHITE << endl;*/  
#endif    
    transformation_ = Transformation * transformation_;
#ifdef PrintMinimizationMatrices 
    cout << BLUE << "AllIter" << endl;
    //Matrix3f tmpMat = transformation_.topLeftCorner(3,3);
    //cout << tmpMat.eulerAngles(0,1,2) << endl;
    cout << transformation_ << WHITE << endl;  
#endif    
    change_ += TransformParams.norm(); 
  }
  
#ifdef PrintMinimizationMatrices 
  cout << "Result: ";
  cout << change_ << endl;
#endif    
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
      Eigen::Vector3f pnt(1.0f*i,1.0f*j,0.0);
      mp.second_point = pnt;
      pnt[0] += 0.1f;
      pnt[1] += 0.2f;
      pnt[2] += 0.3f;
      mp.first_point = rot*pnt;
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
      Eigen::Vector3f pnt(0.f,1.0f*j,1.f*i);
      mp.second_point = pnt;
      pnt[0] += 0.1f;
      pnt[1] += 0.2f;
      pnt[2] += 0.3f;
      mp.first_point = rot*pnt;
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
      Eigen::Vector3f pnt(1.0f*j,0.f,1.f*i);
      mp.second_point = pnt;
      pnt[0] += 0.1f;
      pnt[1] += 0.2f;
      pnt[2] += 0.3f;
      mp.first_point = rot*pnt;
      selected_.push_back(mp);      
    }
  }
  selectedCount_ = selected_.size();
  Minimization();
}

