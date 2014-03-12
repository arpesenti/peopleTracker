#include "mex.h"
#include "matrix.h"
#include <math.h>
#include <omp.h>
#include <stdint.h>

#define MAX_THREAD 32
#define DIST_SHRINK 20 //mm

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{  
  
  if(nrhs<4) 
    mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumInputs",
            "Two group of input points required and their centroids (optionally a threshold).");
  if(nlhs!=1) 
    mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumOutputs",
            "One output required.");
  
  // retrive input parameters
  
  const mxArray *Mpoints1 = prhs[0];
  double *points1 = mxGetPr(Mpoints1);
  
  const mxArray *Mpoints2 = prhs[1];
  double *points2 = mxGetPr(Mpoints2);
  
  int numPoints1 = mxGetN(Mpoints1);
  int numPoints2 = mxGetN(Mpoints2);
  int dim1 = mxGetM(Mpoints1);
  int dim2 = mxGetM(Mpoints2);
  
  const mxArray *Mcentroid1 = prhs[2];
  double *centroid1 = mxGetPr(Mcentroid1);
  
  const mxArray *Mcentroid2 = prhs[3];
  double *centroid2 = mxGetPr(Mcentroid2);
    
  double threshold = 0;
  if(nrhs==5) {
      threshold = *mxGetPr(prhs[4]);
      threshold = threshold*threshold;
  }
  
  if (dim1 != dim2)
      mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumInputs",
            "Points must have the same dimension.");
  
  // compute line between centroids
  double distanceCentroids = (centroid1[0]-centroid2[0])*(centroid1[0]-centroid2[0]) + (centroid1[1]-centroid2[1])*(centroid1[1]-centroid2[1]);
  double m = (centroid1[1]-centroid2[1])/(centroid1[0]-centroid2[0]);
  double mOrt = -1/m;
  double c1 = -mOrt*centroid1[0] + centroid1[1];
  double c2 = -mOrt*centroid2[0] + centroid2[1];
  double a1 = mOrt, a2 = mOrt, b1 = -1, b2 = -1;
  double norm1 = sqrt(a1*a1 + b1*b1);
  double norm2 = sqrt(a2*a2 + b2*b2);
  a1 = a1/norm1;
  b1 = b1/norm1;
  a2 = a2/norm2;
  b2 = b2/norm2;
  if (a1*centroid2[0] + b1*centroid2[1] + c1 < 0) {
    a1 = -a1;
    b1 = -b1;
    c1 = -c1;
  }
  if (a2*centroid1[0] + b2*centroid1[1] + c2 < 0) {
    a2 = -a2;
    b2 = -b2;
    c2 = -c2;
  }
  
  // select points2
  double *selectedPoints2 = (double *)malloc(3*numPoints2*sizeof(double));
  int numSelectedPoints2 = 0;
  for (int j=0; j<numPoints2; j++) {
    if ( a2*points2[3*j] + b2*points2[3*j+1] + c2 > DIST_SHRINK ) {
        selectedPoints2[numSelectedPoints2*3] = points2[j*3];
        selectedPoints2[numSelectedPoints2*3 + 1] = points2[j*3 + 1];
        selectedPoints2[numSelectedPoints2*3 + 2] = points2[j*3 + 2];
        numSelectedPoints2++;
    }
  }
  
  // parallelize computation of distances
  
  double minDist[MAX_THREAD];
  int id1[MAX_THREAD],id2[MAX_THREAD];
  uint8_t numberOfThreads = 1;
  int stopVal = 0;
  int *stop = &stopVal;
    
  #pragma omp parallel
  {
  int threadID = omp_get_thread_num();
  int numThreads = omp_get_num_threads();
  numberOfThreads = numThreads;
  int pointsPerThread = numPoints1/numThreads;
 
  int start = threadID*pointsPerThread;
  int finish = start + pointsPerThread - 1;
  minDist[threadID] = distanceCentroids;
  
  double x1,y1,z1,dist,dx,dy,dz;
  
  for (int i=start; i<=finish; i++) {
    x1 = points1[i*3];
    y1 = points1[i*3+1];
    z1 = points1[i*3+2];
    
    if (a1*x1 + b1*y1 + c1 < DIST_SHRINK ) continue;
    
    for (int j=0; j<numSelectedPoints2; j++) {
        dx = (x1-selectedPoints2[j*3]);
        dy = (y1-selectedPoints2[j*3+1]);
        dz = (z1-selectedPoints2[j*3+2]);
        dist = dx*dx+dy*dy+dz*dz;
        if (dist < minDist[threadID]) {
            minDist[threadID] = dist;
        }
    }
    if (*stop) {
        break;
    }
    if (minDist[threadID] < threshold) {
        *stop = 1;
        break;
    }
  }
  }
  
  // output parameters
  
  plhs[0] = mxCreateDoubleMatrix(1,1,mxREAL);
  double *min = mxGetPr(plhs[0]);
  min[0] = distanceCentroids;

  for (int k = 0; k<numberOfThreads; k++) {
    if (minDist[k] < min[0]) {
        min[0] = minDist[k];
    }
  }
  min[0] = sqrt(min[0]);
  
  free(selectedPoints2);
}
