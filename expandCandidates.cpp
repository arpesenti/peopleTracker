#include "mex.h"
#include "nanoflann.hpp"
#include <queue>
#include <cstring>
#include <chrono>
#include <cstdint>
#include <cmath>

#define LEGCUTHEIGHT 500 // height in mm over the floor to stop leg expansion for classification

// DBSCAN parameters
#define DBSCANRADIUS 35 // dbscan radius in mm
#define DBSCANNNEIGH 3 // dbscan minimum number of neighbors

// rectified depth image (silhouette) parameters
#define SWIDTH 400 // width in mm
#define SHEIGHT 500 // height in mm
#define SDEPTH 600 // max depth in mm
#define SSIDE 10 // discetization size in mm

using namespace std;
using namespace chrono;
using namespace nanoflann;

// point cloud adapter for building the kd-tree (nanoflann)
struct PointCloud
{
    double *pts;
    size_t numPoints;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return numPoints; }

	// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
	inline double kdtree_distance(const double *p1, const size_t idx_p2, size_t size) const
	{
        const double d0=p1[0]-pts[idx_p2*3];
		const double d1=p1[1]-pts[idx_p2*3+1];
		const double d2=p1[2]-pts[idx_p2*3+2];
        return d0*d0+d1*d1+d2*d2;
	}

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline double kdtree_get_pt(const size_t idx, int dim) const
	{
		return pts[idx*3+dim];
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX &bb) const { return false; }

};

void expandCandidate(double *footPoints, double *legPoints, double *personPoints, double *centroidsOnFloor, KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PointCloud>, PointCloud, 3> *tree, size_t numFootPoints, size_t *actualNumLegPoints, size_t *actualNumPersonPoints, size_t numCentroidsOnFloor, uint8_t *visited, double *points, size_t idCandidate, uint8_t *colors, uint8_t *legColors, uint8_t *footColors, double *legPointsNormalized, double *silhouette, int numPoints) {
    
    int legCutHeight = LEGCUTHEIGHT;
    
    double eps = DBSCANRADIUS;
    int k = DBSCANNNEIGH;
    int reserveSize = 50;
    
    double xMin=4000, yMin=4000, xMax = 0;
    
    // foot centroid
    double xCenter = centroidsOnFloor[idCandidate*2];
    double yCenter = centroidsOnFloor[idCandidate*2+1];
    
    // foot angle on the scene
    double alpha = acos(yCenter/sqrt(xCenter*xCenter + yCenter*yCenter));
    if (xCenter<0) 
        alpha = -alpha;
    double cosAlpha = cos(alpha);
    double sinAlpha = sin(alpha);
    
    bool enabledColor = true;
    if (colors == NULL || legColors == NULL || footColors == NULL)
        enabledColor = false;
    
    // copy foot data in leg fields
    memcpy(legPoints, footPoints, numFootPoints*sizeof(double)*3);
    if (enabledColor)
        memcpy(legColors, footColors, numFootPoints*sizeof(uint8_t)*3);
    
    *actualNumLegPoints = numFootPoints;
    
    const double search_radius = static_cast<double>(eps*eps);
    nanoflann::SearchParams params;
    params.sorted = false;
    
    queue<size_t> neigh;
    
    double queryPoint[3];
    
    std::vector<std::pair<size_t,double> >  ret_matches;
    
    // find immediate neighbors of foot points, from which it will start the expansion
    for (int i=0; i<numFootPoints; i++) {
        queryPoint[0] = footPoints[i*3];
        queryPoint[1] = footPoints[i*3+1];
        queryPoint[2] = footPoints[i*3+2];
                
        legPointsNormalized[3*i] = cosAlpha*queryPoint[0]-sinAlpha*queryPoint[1];
        legPointsNormalized[3*i+1] = sinAlpha*queryPoint[0]+cosAlpha*queryPoint[1];
        legPointsNormalized[3*i+2] = queryPoint[2];
        
        // find min and max coordinates for normalize the candidate
        if (legPointsNormalized[3*i] < xMin)
            xMin = legPointsNormalized[3*i];
        if (legPointsNormalized[3*i] > xMax)
            xMax = legPointsNormalized[3*i];
        if (legPointsNormalized[3*i+1] < yMin)
            yMin = legPointsNormalized[3*i+1];
        
        // search neighbors of the current foot point
        ret_matches.clear();
        ret_matches.reserve(reserveSize);
        const size_t nMatches = (*tree).radiusSearch(queryPoint, search_radius, ret_matches, params);
        
        if (nMatches >= k) {
            // border connected with outer points
            for(int j=0;j<nMatches;j++) {
                size_t id = ret_matches[j].first;
                if(visited[id] == 0) {
                    // if not already visited add to expansion queue
                    neigh.push(id);
                    visited[id] = 1;
                    legPoints[(*actualNumLegPoints)*3] = points[3*id];
                    legPoints[(*actualNumLegPoints)*3+1] = points[3*id+1]; 
                    legPoints[(*actualNumLegPoints)*3+2] = points[3*id+2];
                    
                    legPointsNormalized[(*actualNumLegPoints)*3] = cosAlpha*points[3*id]-sinAlpha*points[3*id+1];
                    legPointsNormalized[(*actualNumLegPoints)*3+1] = sinAlpha*points[3*id]+cosAlpha*points[3*id+1];
                    legPointsNormalized[(*actualNumLegPoints)*3+2] = points[3*id+2];

                    if (legPointsNormalized[(*actualNumLegPoints)*3] < xMin)
                        xMin = legPointsNormalized[(*actualNumLegPoints)*3];
                    if (legPointsNormalized[(*actualNumLegPoints)*3] > xMax)
                        xMax = legPointsNormalized[(*actualNumLegPoints)*3];
                    if (legPointsNormalized[(*actualNumLegPoints)*3+1] < yMin)
                        yMin = legPointsNormalized[(*actualNumLegPoints)*3+1];
                    
                    if (enabledColor) {
                        legColors[(*actualNumLegPoints)*3] = colors[3*id];
                        legColors[(*actualNumLegPoints)*3+1] = colors[3*id+1]; 
                        legColors[(*actualNumLegPoints)*3+2] = colors[3*id+2];
                    }
                    
                    *actualNumLegPoints += 1;
                }
            }
        }
    }

    // copy leg points into person points
    memcpy(personPoints, legPoints, (*actualNumLegPoints)*sizeof(double)*3);
    *actualNumPersonPoints = *actualNumLegPoints;
        
    double x,y,z;
    
    // expand the leg and the person starting from the just found border points
    while(!neigh.empty()) {
        size_t id = neigh.front();
        neigh.pop();

        queryPoint[0] = points[id*3];
        queryPoint[1] = points[id*3+1];
        queryPoint[2] = points[id*3+2];
        
        // neighbors search
        ret_matches.clear();
        ret_matches.reserve(reserveSize);
        const size_t nMatches = (*tree).radiusSearch(queryPoint, search_radius, ret_matches, params);

        if (nMatches >= k+1) {
            // core point
            for(int j=0;j<nMatches;j++) {
                size_t idx = ret_matches[j].first;
                if(visited[idx] == 0) {
                    // check if belong to the current voronoi region
                    double minDist = 4000*4000;
                    int label = -1;
                    x = points[idx*3];
                    y = points[idx*3+1];
                    z = points[idx*3+2];
                    for (int t=0; t<numCentroidsOnFloor; t++) {
                        // Voronoi
                        double dist = (x-centroidsOnFloor[t*2])*(x-centroidsOnFloor[t*2]) + (y-centroidsOnFloor[t*2+1])*(y-centroidsOnFloor[t*2+1]);
                        if (dist<minDist) {
                            minDist = dist;
                            label = t;
                        }
                    }
                    if (label == idCandidate) {
                        // add to current expansion
                        neigh.push(idx);
                        visited[idx] = 1;
                        if (*actualNumPersonPoints >= numPoints+numFootPoints) {
                            //mexPrintf("numFootPoints: %d numPoints: %d actualPerson: %d actualLeg: %d\n", numFootPoints, numPoints, *actualNumPersonPoints, *actualNumLegPoints);
                            continue;
                        }
                        
                        // add to person points
                        personPoints[(*actualNumPersonPoints)*3] = x;
                        personPoints[(*actualNumPersonPoints)*3+1] = y; 
                        personPoints[(*actualNumPersonPoints)*3+2] = z;

                        *actualNumPersonPoints += 1;

                        if (z < legCutHeight) {
                            // add to leg points
                            legPoints[(*actualNumLegPoints)*3] = x;
                            legPoints[(*actualNumLegPoints)*3+1] = y; 
                            legPoints[(*actualNumLegPoints)*3+2] = z;
                            
                            legPointsNormalized[(*actualNumLegPoints)*3] = cosAlpha*x-sinAlpha*y;
                            legPointsNormalized[(*actualNumLegPoints)*3+1] = sinAlpha*x+cosAlpha*y;
                            legPointsNormalized[(*actualNumLegPoints)*3+2] = z;

                            if (legPointsNormalized[(*actualNumLegPoints)*3] < xMin)
                                xMin = legPointsNormalized[(*actualNumLegPoints)*3];
                            if (legPointsNormalized[(*actualNumLegPoints)*3] > xMax)
                                xMax = legPointsNormalized[(*actualNumLegPoints)*3];
                            if (legPointsNormalized[(*actualNumLegPoints)*3+1] < yMin)
                                yMin = legPointsNormalized[(*actualNumLegPoints)*3+1];
                            
                            
                            if (enabledColor) {
                                legColors[(*actualNumLegPoints)*3] = colors[idx*3];
                                legColors[(*actualNumLegPoints)*3+1] = colors[idx*3+1]; 
                                legColors[(*actualNumLegPoints)*3+2] = colors[idx*3+2];
                            }

                            *actualNumLegPoints += 1;
                        }

                    }
                    
                }
            }
        }
    }
    
    // normalize legPoints and create silhouette
    int srow = SHEIGHT/SSIDE;
    int scol = SWIDTH/SSIDE;
    int sdepth = SDEPTH/SSIDE;
    double xoff = (SWIDTH-(xMax-xMin))/2;
    uint8_t *depthIndex = (uint8_t *)malloc(srow*scol*sizeof(uint8_t));
    uint8_t *count = (uint8_t *)calloc(srow*scol,sizeof(uint8_t));
    int currentRow, currentCol, currentDepth, linearIdx;
    
    // compute linear idx where each leg points belong
    for (int i=0; i<*actualNumLegPoints; i++) {
        legPointsNormalized[3*i] -= xMin;
        legPointsNormalized[3*i+1] -= yMin;
        
        currentRow = srow-1 - (int)(legPointsNormalized[3*i+2]/SSIDE);
        currentCol = (int)((legPointsNormalized[3*i]+xoff)/SSIDE);
        currentDepth = (int)(legPointsNormalized[3*i+1]/SSIDE)+1;
        if (currentRow<0 || currentCol<0 || currentDepth<0 || currentRow>srow-1 || currentCol>scol-1 || currentDepth>sdepth-1) continue;
        
        linearIdx = currentCol*srow + currentRow;
        if (count[linearIdx] == 0 || currentDepth < depthIndex[linearIdx]) {
            count[linearIdx] = 1;
            silhouette[linearIdx] = legPointsNormalized[3*i+1];
            depthIndex[linearIdx] = currentDepth;
        } else if (currentDepth == depthIndex[linearIdx]) {
            silhouette[linearIdx] = (silhouette[linearIdx]*count[linearIdx] + legPointsNormalized[3*i+1])/(count[linearIdx] + 1);
            count[linearIdx]++;
        }
    }
    
    // fill silhouette holes
    uint8_t *binary = depthIndex;
    memset(binary, 0, srow*scol*sizeof(uint8_t));
    for (int i=0; i<srow*scol; i++) {
        if (count[i] == 0) {
            silhouette[i] = SDEPTH;
        } else {   
            binary[i] = 2;
        }
    }
    // dilate
    for (int i=1; i<scol-1; i++) {
        for (int j=1; j<srow-1; j++) {
            if (binary[i*srow+j] == 2) {
                if (binary[(i-1)*srow+j] == 0) binary[(i-1)*srow+j] = 3;
                if (binary[(i-1)*srow+j-1] == 0) binary[(i-1)*srow+j-1] = 3;
                if (binary[(i)*srow+j-1] == 0) binary[(i)*srow+j-1] = 3;
                if (binary[(i+1)*srow+j-1] == 0) binary[(i+1)*srow+j-1] = 3;
                if (binary[(i+1)*srow+j] == 0) binary[(i+1)*srow+j] = 3;
                if (binary[(i+1)*srow+j+1] == 0) binary[(i+1)*srow+j+1] = 3;
                if (binary[(i)*srow+j+1] == 0) binary[(i)*srow+j+1] = 3;
                if (binary[(i-1)*srow+j+1] == 0) binary[(i-1)*srow+j+1] = 3;
            }
        }
    }
    // erode
    for (int i=1; i<scol-1; i++) {
        for (int j=1; j<srow-1; j++) {
            if (binary[i*srow+j] >= 2) {
                if (binary[(i-1)*srow+j] == 0) { 
                    binary[(i)*srow+j] = 1;
                    continue;
                }
                if (binary[(i)*srow+j-1] == 0) { 
                    binary[(i)*srow+j] = 1;
                    continue;
                }
                if (binary[(i+1)*srow+j-1] == 0) { 
                    binary[(i)*srow+j] = 1;
                    continue;
                }
                if (binary[(i+1)*srow+j] == 0) { 
                    binary[(i)*srow+j] = 1;
                    continue;
                }
                if (binary[(i+1)*srow+j+1] == 0) { 
                    binary[(i)*srow+j] = 1;
                    continue;
                }
                if (binary[(i)*srow+j+1] == 0) { 
                    binary[(i)*srow+j] = 1;
                    continue;
                }
                if (binary[(i-1)*srow+j+1] == 0) { 
                    binary[(i)*srow+j] = 1;
                    continue;
                }
                if (binary[(i-1)*srow+j-1] == 0) { 
                    binary[(i)*srow+j] = 1;
                    continue;
                }
            }
        }
    }
    // fill hole in correspondence of 3 in binary image with average of non zero neighbors
    double *silhouetteCopy = (double *)malloc(srow*scol*sizeof(double));
    silhouetteCopy = (double *)memcpy(silhouetteCopy, silhouette, srow*scol*sizeof(double));
    int numNeigh;
    double mean;
    for (int i=1; i<scol-1; i++) {
        for (int j=1; j<srow-1; j++) {
            if (binary[i*srow+j] == 3) {
                mean = 0;
                numNeigh = 0;
                if (binary[(i-1)*srow+j] == 2) { 
                    mean += silhouetteCopy[(i-1)*srow+j];
                    numNeigh++;
                }
                if (binary[(i)*srow+j-1] == 2) { 
                    mean += silhouetteCopy[(i)*srow+j-1];
                    numNeigh++;
                }
                if (binary[(i+1)*srow+j-1] == 2) { 
                    mean += silhouetteCopy[(i+1)*srow+j-1];
                    numNeigh++;
                }
                if (binary[(i+1)*srow+j] == 2) { 
                    mean += silhouetteCopy[(i+1)*srow+j];
                    numNeigh++;
                }
                if (binary[(i+1)*srow+j+1] == 2) { 
                    mean += silhouetteCopy[(i+1)*srow+j+1];
                    numNeigh++;
                }
                if (binary[(i)*srow+j+1] == 2) { 
                    mean += silhouetteCopy[(i)*srow+j+1];
                    numNeigh++;
                }
                if (binary[(i-1)*srow+j+1] == 2) { 
                    mean += silhouetteCopy[(i-1)*srow+j+1];
                    numNeigh++;
                }
                if (binary[(i-1)*srow+j-1] == 2) { 
                    mean += silhouetteCopy[(i-1)*srow+j-1];
                    numNeigh++;
                }
                
                silhouette[i*srow+j] = mean/numNeigh;
                
            }
        }
    }
    
    free(depthIndex);
    free(count);
    free(silhouetteCopy);
}


void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
  // parameters (in order): candidates, centroidsOnFloor, voxelGridPoints, voxelGridColors
  
  if(nrhs!=4) 
    mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumInputs",
            "Four inputs required.");
  if(nlhs!=1) 
    mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumOutputs",
            "One output required.");
  
  const mxArray *inputCandidates = prhs[0];
  const mxArray *centroidsOnFloorM = prhs[1];
  const mxArray *voxelGridPoints = prhs[2];
  const mxArray *voxelGridColors = prhs[3];
  
  // check if color data is needed (plot enabled)
  bool enabledColor = false;
  if (mxGetM(voxelGridColors) != 0) enabledColor = true;
  
  uint8_t *colors = (uint8_t *)mxGetData(voxelGridColors);
  
  mxArray *candidates = mxDuplicateArray(inputCandidates); // array of candidate structs
  
  int srow = SHEIGHT/SSIDE; // number of rows in silhouette
  int scol = SWIDTH/SSIDE; // number of columns in silhouette
  
  // build kd-tree of all point cloud
  PointCloud pointCloud;
  pointCloud.pts = mxGetPr(voxelGridPoints); 
  pointCloud.numPoints = mxGetN(voxelGridPoints);
  KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PointCloud>, PointCloud, 3 /* dim */> tree(3 /*dim*/, pointCloud, KDTreeSingleIndexAdaptorParams(20 /* max leaf */) );
  tree.buildIndex();
  
  size_t numCandidates = mxGetN(candidates);
  size_t numPoints = mxGetN(voxelGridPoints);
  
  double *centroidsOnFloor = mxGetPr(centroidsOnFloorM);
  size_t numCentroidsOnFloor = mxGetN(centroidsOnFloorM);
  
  // allocations for filling candidate structs 
  double **footPointsPtrs = (double **)malloc(numCandidates*sizeof(double *));
  double **legPointsPtrs = (double **)malloc(numCandidates*sizeof(double *));
  double **legPointsNormalizedPtrs = (double **)malloc(numCandidates*sizeof(double *));
  double **silhouettePtrs = (double **)malloc(numCandidates*sizeof(double *));
  double **personPointsPtrs = (double **)malloc(numCandidates*sizeof(double *));
  size_t *numFootPointsPtrs = (size_t *)malloc(numCandidates*sizeof(size_t));
  size_t *actualNumLegPointsPtrs = (size_t *)malloc(numCandidates*sizeof(size_t));
  size_t *actualNumPersonPointsPtrs = (size_t *)malloc(numCandidates*sizeof(size_t));
  uint8_t **legColorsPtrs;
  uint8_t **footColorsPtrs;
  if (enabledColor) {
      legColorsPtrs = (uint8_t **)malloc(numCandidates*sizeof(uint8_t *));
      footColorsPtrs = (uint8_t **)malloc(numCandidates*sizeof(uint8_t *));
  }
  
  const mwSize colorsDim[] = {3,(mwSize)numPoints};
  
  for (int i=0; i < numCandidates; i++) {
      footPointsPtrs[i] = mxGetPr(mxGetField(candidates,i,"footPoints"));
      mxArray *footPoints = mxGetField(candidates,i,"footPoints");
      numFootPointsPtrs[i] = (size_t)mxGetN(footPoints);
             
      mxArray *legPoints = mxCreateDoubleMatrix(3, numPoints+numFootPointsPtrs[i], mxREAL); 
      mxSetField(candidates, i, "legPoints", legPoints);
      legPointsPtrs[i] = mxGetPr(legPoints);
      
      mxArray *legPointsNormalized = mxCreateDoubleMatrix(3, numPoints+numFootPointsPtrs[i], mxREAL); 
      mxSetField(candidates, i, "legPointsNormalized", legPointsNormalized);
      legPointsNormalizedPtrs[i] = mxGetPr(legPointsNormalized);
      
      mxArray *silhouette = mxCreateDoubleMatrix(srow, scol, mxREAL); 
      mxSetField(candidates, i, "silhouette", silhouette);
      silhouettePtrs[i] = mxGetPr(silhouette);
      
      mxArray *personPoints = mxCreateDoubleMatrix(3, numPoints+numFootPointsPtrs[i], mxREAL); 
      mxSetField(candidates, i, "allPoints", personPoints);
      personPointsPtrs[i] = mxGetPr(personPoints);
      
      if (enabledColor) {
          footColorsPtrs[i] = (uint8_t *)mxGetData(mxGetField(candidates,i,"footColors"));

          const mwSize colorsDim[] = {3,(mwSize)(numPoints+numFootPointsPtrs[i])};
          mxArray *legColors = mxCreateNumericArray(2, colorsDim, mxUINT8_CLASS, mxREAL); 
          mxSetField(candidates, i, "legColors", legColors);
          legColorsPtrs[i] = (uint8_t *)mxGetData(legColors);
      }
      
  }
  
  // array for points already considered for expansion
  uint8_t *visited = (uint8_t *)calloc(numPoints, sizeof(uint8_t));
      
  // expand candidates in parallel
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

          if (enabledColor)
            expandCandidate(footPointsPtrs[id], legPointsPtrs[id], personPointsPtrs[id], centroidsOnFloor, &tree, numFootPointsPtrs[id], &actualNumLegPointsPtrs[id], &actualNumPersonPointsPtrs[id], numCentroidsOnFloor, visited, pointCloud.pts, id, colors, legColorsPtrs[id], footColorsPtrs[id], legPointsNormalizedPtrs[id], silhouettePtrs[id], numPoints);
          else
            expandCandidate(footPointsPtrs[id], legPointsPtrs[id], personPointsPtrs[id], centroidsOnFloor, &tree, numFootPointsPtrs[id], &actualNumLegPointsPtrs[id], &actualNumPersonPointsPtrs[id], numCentroidsOnFloor, visited, pointCloud.pts, id, NULL, NULL, NULL, legPointsNormalizedPtrs[id], silhouettePtrs[id], numPoints);
      }
  }
  
  // resize arrays in candidate structs, freeing unused memory
  for (int i=0; i < numCandidates; i++) {
      mxArray *legPoints = mxGetField(candidates,i,"legPoints");
      mxSetN(legPoints, actualNumLegPointsPtrs[i]);
      mxArray *legPointsNormalized = mxGetField(candidates,i,"legPointsNormalized");
      mxSetN(legPointsNormalized, actualNumLegPointsPtrs[i]);
      mxArray *personPoints = mxGetField(candidates,i,"allPoints");
      mxSetN(personPoints, actualNumPersonPointsPtrs[i]);
      if (enabledColor) {
          mxArray *legColors = mxGetField(candidates,i,"legColors");
          mxSetN(legColors, actualNumLegPointsPtrs[i]);
      }
  }
  
  plhs[0] = candidates;
  free(footPointsPtrs);
  free(legPointsPtrs);
  free(legPointsNormalizedPtrs);
  free(personPointsPtrs);
  free(numFootPointsPtrs);
  free(actualNumPersonPointsPtrs);
  free(actualNumLegPointsPtrs);
  free(silhouettePtrs);
  free(visited);
  
  if (enabledColor) {
      free(footColorsPtrs);
      free(legColorsPtrs);
  }
  
}
