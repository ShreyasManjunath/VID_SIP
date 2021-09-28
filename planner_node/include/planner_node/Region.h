#ifndef __REGION_H__
#define __REGION_H__

#include <stdlib.h>

namespace VID
{
    class region
    {
        public:
            int numDimensions;
            double *center;
            double *size;
            int occupied;
            region();
            ~region();
            int setNumDimensions (int numDimensionsIn);
    };
}


#endif