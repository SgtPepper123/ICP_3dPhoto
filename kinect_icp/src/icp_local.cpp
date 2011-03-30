
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
  ROS_INFO("IcpLocal::Compute");
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
		
#define SelectionAmount 100
		
void IcpLocal::Selection()
{
  srand(time(NULL));
  int i = 0;
  int count = first_->points.size();
  selected_.clear();
  selected_.reserve(SelectionAmount);
  while(i < SelectionAmount)
  {
    int index = rand()%count;
    float x = first_->points[index].x;
    if(x==x){    
      selected_.push_back(make_pair(index,0));
      cout << index << endl;
      
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
