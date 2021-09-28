#include "planner_node/Region.h"

using namespace std;

VID::region::region()
{
    this->numDimensions = 0;
    this->center = NULL;
    this->size = NULL;
}

VID::region::~region ()
{
    if (this->center)
        delete [] this->center;
    if (this->size)
        delete [] this->size;
}

int VID::region::setNumDimensions (int numDimensionsIn)
{
  this->numDimensions = numDimensionsIn;
  
  if (this->center)
    delete [] this->center;
  this->center = new double[this->numDimensions];
  
  if (this->size)
    delete [] this->size;
  this->size = new double[this->numDimensions];
  
  return 1;
}