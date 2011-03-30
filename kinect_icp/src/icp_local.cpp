
#include "kinect_icp/icp_local.h"

using namespace kinect_icp;

IcpLocal::IcpLocal(PCloud* first, PCloud* second)
: First_(first)
, Second_(second)
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
  BOOST_FOREACH (pcl::PointXYZRGB& pt, first_->points) {
    BOOST_FOREACH (pcl::PointXYZRGB& pt, selected_->points) {

    }
  }
}

void IcpLocal::Matching()
{
	
}

void IcpLocal::Rejecting()
{
	
}

float IcpLocal::Minimization()
{
  return 0.f;
}
