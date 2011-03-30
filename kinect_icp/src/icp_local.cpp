
#include "kinect_icp/icp_local.h"

using namespace kinect_icp;

IcpLocal::IcpLocal(const PCloud::ConstPtr& first, const PCloud::ConstPtr& second)
: m_First(first)
, m_Second(second)
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
