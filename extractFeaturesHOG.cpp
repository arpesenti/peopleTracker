#include "mex.h"
#include <cstring>
#include <cstdint>
#include <cmath>

// HOG parameters
// block size = 1
// signed gradient (from 0 to 360)
#define VPATCHSIZE 10 // vertical cell size
#define HPATCHSIZE 10 // horizontal cell size
#define NBINS 9 // HOG number of bins in histograms

#define SDEPTH 600 // maximum depth

#define PI 3.14159265359

using namespace std;

void hog(double *silhouette, double *features, int nPatchV, int nPatchH, int srow, int scol) {
    
    // allocate memory for gradient and magnitude of gradient
    double *angles = (double *)calloc(srow*scol,sizeof(double));
    double *magnit = (double *)calloc(srow*scol,sizeof(double));
    
    // find maxDepth for silhouette normalization
    double maxDepth = 0, depth;
    for (int j=0; j<scol; j++) {
        for (int i=0; i<srow; i++) {
            depth = silhouette[i+j*srow];
            if ((int)depth != SDEPTH && depth > maxDepth) maxDepth = depth;
        }
    }
    
    double *silhouetteNormalized = (double *)malloc(sizeof(double)*srow*scol);
    memcpy(silhouetteNormalized, silhouette, sizeof(double)*srow*scol);
    
    // normalize silhouette
    for (int j=0; j<scol; j++) {
        for (int i=0; i<srow; i++) {
            if ((int)silhouette[i+j*srow] == SDEPTH )
                silhouetteNormalized[i+j*srow] = 0;
            else
                silhouetteNormalized[i+j*srow] = (maxDepth - silhouette[i+j*srow]);
        }
    }
    
    // compute gradient and gradient magnitude
    double dx, dy;
    for (int j=1; j<scol-1; j++) {
        for (int i=1; i<srow-1; i++) {
            dx = silhouetteNormalized[i + (j+1)*srow] - silhouetteNormalized[i + (j-1)*srow];
            dy = silhouetteNormalized[i-1 + j*srow] - silhouetteNormalized[i+1 + j*srow];
            angles[i+j*srow] = atan2(dy,dx);
            magnit[i+j*srow] = sqrt(dx*dx+dy*dy);
        }
    }
    
    // compute and normalize a histogram for every cell
    int histOffset;
    int patchNum = 0, numBin, id;
    double angle, sector = 2*PI/NBINS, norm; 
    for (int j=0; j<nPatchH; j++) {
        for (int i=0; i<nPatchV; i++) {
            histOffset = patchNum*NBINS;
            for (int s=0; s<HPATCHSIZE; s++) {
                for (int t=0; t<VPATCHSIZE; t++) {
                    id = i*VPATCHSIZE+t + (j*HPATCHSIZE+s)*srow;
                    angle = angles[id]+PI; // signed gradient, from 0 to 2PI
                    numBin = (int)(angle/sector);
                    if (numBin >= NBINS) numBin = NBINS-1;
                    features[histOffset+numBin] += magnit[id];
                }
            }
            
            // block size is 1, normalize single cells
            norm = 0;
            for (int k=0; k<NBINS; k++) {
                norm += features[histOffset+k]*features[histOffset+k];
            }
            norm = sqrt(norm);
            
            for (int k=0; k<NBINS; k++) {
                if (norm > 1)
                    features[histOffset+k] = features[histOffset+k]/norm;
                else
                    features[histOffset+k] = 0; // histogram norm too small, clipping
            }
            
            patchNum++;
        }
    }
    free(silhouetteNormalized);
    free(angles);
    free(magnit);
}


void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
  // parameters: candidates
  
  if(nrhs!=1) 
    mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumInputs",
            "One inputs required.");
  if(nlhs!=1) 
    mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumOutputs",
            "One output required.");
  
  const mxArray *candidates = prhs[0];
  
  size_t numCandidates = mxGetN(candidates);
  
  if (numCandidates == 0) {
      // no candidates, return empty matrix
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
      return;
  }
  
  double **silhouettePtrs = (double **)malloc(numCandidates*sizeof(double *));
  
  mxArray *firstSilhouette = mxGetField(candidates, 0, "silhouette");
  int srow =  mxGetM(firstSilhouette);
  int scol = mxGetN(firstSilhouette);
  silhouettePtrs[0] = mxGetPr(firstSilhouette);
  
  // number of vertical and horizontal cells
  int nPatchV = srow/VPATCHSIZE;
  int nPatchH = scol/HPATCHSIZE;
  
  // allocate memory for features. One colum for each candidate
  mxArray *features = mxCreateDoubleMatrix(nPatchV*nPatchH*NBINS,numCandidates,mxREAL);
  double *featuresPtr = mxGetPr(features);
  
  for (int i=1; i < numCandidates; i++) {
      // read legs rectified depth images (silhouette)
      mxArray *silhouette = mxGetField(candidates, i, "silhouette"); 
      silhouettePtrs[i] = mxGetPr(silhouette);
  }
  
  // parallel extract of HOG features
  int freeCandidate = 0;
  #pragma omp parallel 
  {
      int id;
      while (true) {
          #pragma omp critical 
          {
            id = freeCandidate;
            freeCandidate++;
          }

          if (id >= numCandidates)
              break;

          hog(silhouettePtrs[id], &featuresPtr[id*nPatchV*nPatchH*NBINS], nPatchV, nPatchH, srow, scol);
      }
  }

  plhs[0] = features;
  free(silhouettePtrs);
}
