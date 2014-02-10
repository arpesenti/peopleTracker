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
  // --- PARAMETERS ---
  uint16_t maxDepth = 3500; // mm
  double voxelDim = 10; // mm
  double sideLengthX = 6000/voxelDim; //in voxel
  double sideLengthY = 6000/voxelDim; //in voxel   
  // ------------------
  
  
  // retrieve input data
  const mxArray *Mdepth = prhs[0];
  uint16_t *depth = (uint16_t *)mxGetData(Mdepth);
  
  const mxArray *Mrgb = prhs[1];
  bool enabledColor = false;
  if (mxGetM(Mrgb) != 0) enabledColor = true;
  uint8_t *rgb = (uint8_t *)mxGetData(Mrgb);
  
  int numRows = mxGetM(Mdepth);
  int numCols = mxGetN(Mdepth);
  int numPoints = numRows*numCols;
  
  uint64_t prime;
  register double fx,fy,cx,cy; // sensor intrinsic parameters
  if (numPoints == 640*480) {
      // high resolution
      prime = 20507;
      fx = 1/525.0;
      fy = 1/525.0;
      cx = 319.5;
      cy = 239.5;
  } else if (numPoints == 320*240) {
      // low resolution
      prime = 11113;
      fx = 1/(525.0/2);
      fy = 1/(525.0/2);
      cx = 159.75;
      cy = 119.75;
  } else {
      mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumInputs",
            "Resolution not supported.");
  }
  
  if (enabledColor && (numRows != mxGetM(Mrgb) || numCols != mxGetN(Mrgb)/3))
      mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumInputs",
            "Depth and RGB must be of the same resolution.");
  
  if(nrhs!=2) 
    mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumInputs",
            "Depth and RGB input required.");
  if(nlhs!=2) 
    mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumOutputs",
            "Two output required.");
  
  // initialize output data
  mwSize sizeColors[] = {3,(mwSize)numPoints};
  mwSize sizeCounts[] = {1,(mwSize)numPoints};
  
  mxArray *Mpoints = mxCreateDoubleMatrix(3,numPoints, mxREAL);
  mxArray *Mcounts = mxCreateNumericArray(2,sizeCounts, mxUINT8_CLASS, mxREAL);
  double *points = mxGetPr(Mpoints);
  uint8_t *counts = (uint8_t *)mxGetData(Mcounts);
  
  mxArray *Mcolors;
  uint8_t *colors;
  if (enabledColor) {
      Mcolors = mxCreateNumericArray(2,sizeColors, mxUINT8_CLASS, mxREAL);
      colors = (uint8_t *)mxGetData(Mcolors);
  } else {
      mwSize sizeColorsEmpty[] = {0,0};
      Mcolors = mxCreateNumericArray(2,sizeColorsEmpty, mxUINT8_CLASS, mxREAL);
  }

  uint32_t offsets[MAX_THREAD];
  uint8_t numberOfThreads = 1;
  
  register double invVoxelDim = 1/voxelDim;
  
  #pragma omp parallel shared(numberOfThreads, depth, rgb, points, offsets, voxelDim, sideLengthX, sideLengthY)
  {
  int threadID = omp_get_thread_num();
  int numThreads = omp_get_num_threads();
  numberOfThreads = numThreads;
  int pointsPerThread = numPoints/numThreads;
  
  uint32_t freeOffset = threadID*pointsPerThread;
  
  std::unordered_map<uint32_t, uint32_t> voxel;
  voxel.reserve(numPoints/numThreads);
 
  int start = threadID*pointsPerThread;
  int finish = start + pointsPerThread - 1;
  
  double x,y,z;
  uint32_t offset, count;
  double invCount;
  
  if (enabledColor) {
      for (int i=start; i<=finish; i++) {
        if (depth[i] <= 0 || depth[i] > maxDepth)
            continue;

        z = depth[i];
        x = -(i/numRows+1 - cx)*z*fx; // assuming depth not mirrored
        y = -((i%numRows)+1 - cy)*z*fy;

        uint32_t voxelID = (uint32_t)(x*invVoxelDim+sideLengthX/2) + (uint32_t)(y*invVoxelDim+sideLengthY/2)*sideLengthX + (uint32_t)(z*invVoxelDim)*sideLengthX*sideLengthY;

        if (voxel.count(voxelID) == 0) {
            voxel[voxelID] = freeOffset;
            colors[3*freeOffset] = rgb[i];
            colors[3*freeOffset+1] = rgb[i+numPoints];
            colors[3*freeOffset+2] = rgb[i+2*numPoints];
            freeOffset++;
        }

        offset = voxel[voxelID];
        count = (uint32_t)counts[offset];
        invCount = 1/(count+1.0);

        // average among the points falling in the same voxel
        points[3*offset] = (points[3*offset]*count + x) * invCount;
        points[3*offset+1] = (points[3*offset+1]*count + y) * invCount;
        points[3*offset+2] = (points[3*offset+2]*count + z) * invCount;

        counts[offset]++;
      }
  } else {
      for (int i=start; i<=finish; i++) {
        if (depth[i] <= 0 || depth[i] > maxDepth)
            continue;

        z = depth[i];
        x = -(i/numRows+1 - cx)*z*fx; // assuming depth not mirrored
        y = -((i%numRows)+1 - cy)*z*fy;

        uint32_t voxelID = (uint32_t)(x*invVoxelDim+sideLengthX/2) + (uint32_t)(y*invVoxelDim+sideLengthY/2)*sideLengthX + (uint32_t)(z*invVoxelDim)*sideLengthX*sideLengthY;

        if (voxel.count(voxelID) == 0) {
            voxel[voxelID] = freeOffset;
            freeOffset++;
        }

        offset = voxel[voxelID];
        count = (uint32_t)counts[offset];
        invCount = 1/(count+1.0);

        // average among the points falling in the same voxel
        points[3*offset] = (points[3*offset]*count + x) * invCount;
        points[3*offset+1] = (points[3*offset+1]*count + y) * invCount;
        points[3*offset+2] = (points[3*offset+2]*count + z) * invCount;

        counts[offset]++;
      }
  }

  offsets[threadID] = freeOffset-threadID*pointsPerThread; 
  }
  
  
  int start = 0;
  for (int i=1; i<numberOfThreads; i++) {
    start += offsets[i-1];
    if (enabledColor)
        memcpy(&colors[3*start + 1], &colors[3*i*numPoints/numberOfThreads + 1], sizeof(uint8_t)*offsets[i]*3);
    memcpy(&points[3*start + 1], &points[3*i*numPoints/numberOfThreads + 1], sizeof(double)*offsets[i]*3); 
  }
  
  if (enabledColor)
      mxSetN(Mcolors,start+offsets[numberOfThreads-1]);
  
  mxSetN(Mpoints,start+offsets[numberOfThreads-1]);
  
  plhs[0] = Mpoints;
  plhs[1] = Mcolors;
    
}
