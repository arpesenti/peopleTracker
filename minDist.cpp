#include "mex.h"
#include "matrix.h"
#include <math.h>
#include <cstring>
#include <map>
#include <unordered_map>
#include <omp.h>
#include <ctime>
#include <chrono>

#define MAX_THREAD 32


void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{  
  
  if(nrhs<2) 
    mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumInputs",
            "Two group of input points required (optionally a threshold).");
  if(nlhs!=3) 
    mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumOutputs",
            "Three output required.");
  
  // retrive input parameters
  
  const mxArray *Mpoints1 = prhs[0];
  double *points1 = mxGetPr(Mpoints1);
  
  const mxArray *Mpoints2 = prhs[1];
  double *points2 = mxGetPr(Mpoints2);
  
  int numPoints1 = mxGetN(Mpoints1);
  int numPoints2 = mxGetN(Mpoints2);
  int dim1 = mxGetM(Mpoints1);
  int dim2 = mxGetM(Mpoints2);
  
  double maxDist = 5000*5000;
  
  double threshold = 0;
  if(nrhs==3) {
      threshold = *mxGetPr(prhs[2]);
      threshold = threshold*threshold;
  }
  
  if (dim1 != dim2)
      mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumInputs",
            "Points must have the same dimension.");
  
  // parallelize computation of distances
  
  double minDist[MAX_THREAD];
  int id1[MAX_THREAD],id2[MAX_THREAD];
  uint8_t numberOfThreads = 1;
  int stop = 0;
    
  #pragma omp parallel
  {
  int threadID = omp_get_thread_num();
  int numThreads = omp_get_num_threads();
  numberOfThreads = numThreads;
  int pointsPerThread = numPoints1/numThreads;
 
  int start = threadID*pointsPerThread;
  int finish = start + pointsPerThread - 1;
  minDist[threadID] = maxDist;
  
  double x1,y1,z1,dist,dx,dy,dz;
  
  for (int i=start; i<=finish; i++) {
    x1 = points1[i*3];
    y1 = points1[i*3+1];
    z1 = points1[i*3+2];
    for (int j=0; j<numPoints2; j++) {
        dx = (x1-points2[j*3]);
        dy = (y1-points2[j*3+1]);
        dz = (z1-points2[j*3+2]);
        dist = dx*dx+dy*dy+dz*dz;
        if (dist < minDist[threadID]) {
            minDist[threadID] = dist;
            id1[threadID] = i;
            id2[threadID] = j;
        }
    }
    if (stop)
        break;
    if (minDist[threadID] < threshold) {
        stop = 1;
        break;
    }
  }
  }
  
  // output parameters
  
  plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
  double *min = mxGetPr(plhs[0]);
  min[0] = maxDist;
  plhs[1] = mxCreateDoubleMatrix(1,1,mxREAL);
  double *i = mxGetPr(plhs[1]);
  plhs[2] = mxCreateDoubleMatrix(1,1,mxREAL);
  double *j = mxGetPr(plhs[2]);
  
  for (int k = 0; k<numberOfThreads; k++) {
    if (minDist[k] < min[0]) {
        min[0] = minDist[k];
        i[0] = id1[k]+1;
        j[0] = id2[k]+1;
    }
  }
  min[0] = sqrt(min[0]);
}
