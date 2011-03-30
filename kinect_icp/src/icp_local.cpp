
#include "kinect_icp/icp_local.h"
#include "time.h"

using namespace kinect_icp;
using namespace std;

IcpLocal::IcpLocal(PCloud* first, PCloud* second)
: first_(first)
, second_(second)
{
}
	

void IcpLocal::Compute(/*SomeMatrixClass initialTransformation*/)
{
  float error = std::numeric_limits<float>::max();
  float old_error;
  do
  {
    old_error = error;
    Selection();
    Matching();
    Rejecting();
    error = Minimization();
  } while (abs(old_error-error) > ITERATION_THRESHOLD);
}

		
/*SomeMatrixClass IcpLocal::GetTransformation()
{
	return asdkasdkljas;
}
*/  
		
void IcpLocal::Selection()
{
  //srand()
}

void IcpLocal::Matching()
{
  int imax = selected_.size();
  for (int i=0; i<imax; i++)
  {
    selected_[i].distance = numeric_limits<float>::max();

    int jmax = first_->points.size();
    for (int j=0; j<jmax; j++)
    {
      float dist_x = first_->points[j].x - second_->points[selected_[i].first_index].x;
      float dist_y = first_->points[j].y - second_->points[selected_[i].first_index].y;
      float dist_z = first_->points[j].z - second_->points[selected_[i].first_index].z;

      float dist = dist_x*dist_x + dist_y*dist_y + dist_z*dist_z;
      if (dist < selected_[i].distance) {
        selected_[i].distance = dist;
        selected_[i].second_index = j;
      }
    }
  }
}

void IcpLocal::Rejecting()
{
	
}

float IcpLocal::Minimization()
{
  return 0.f;
}
