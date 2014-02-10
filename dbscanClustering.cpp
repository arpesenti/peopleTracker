#include "mex.h"
#include "nanoflann.hpp"
#include <queue>
#include <cstring>
#include <chrono>
#include <cstdint>

#define DIM 3

using namespace std;
using namespace chrono;
using namespace nanoflann;

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

void dbscan(const mxArray *x, double *classes, double *types, double k, double eps, size_t numPoints, size_t dim)
{
  int C = 0; /* class id assigned to the points in the same cluster */
  int reserveSize = 50; /* allocation dimension for efficiency reasons */
  
  uint8_t *visited = (uint8_t *)calloc(numPoints,sizeof(uint8_t));
  
  double *points = mxGetPr(x);
  
  PointCloud pc;
  pc.pts = points;
  pc.numPoints = numPoints;
  
  if(dim != DIM)
      mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumInputs",
            "Dimensionality error");
  
  // create KDTree
  KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PointCloud>, PointCloud, DIM /* dim */> index(DIM /*dim*/, pc, KDTreeSingleIndexAdaptorParams(20 /* max leaf */) );
  
  index.buildIndex();
    
  const double search_radius = static_cast<double>(eps*eps);
  nanoflann::SearchParams params;
  params.sorted = false;

  double queryPoint[DIM];
  
  queue<int> neigh;
  std::vector<std::pair<size_t,double> > ret_matches;

  int n=0;
  int matches = 0;
  
  // start clusterization
  for(int i=0; i<numPoints; i++) {
     if(visited[i] == 0) {
        
        queryPoint[0] = points[i*3];
        queryPoint[1] = points[i*3+1];
        queryPoint[2] = points[i*3+2];
    
        ret_matches.clear();
        ret_matches.reserve(reserveSize);
        
        // find the number of neighbors of current processed point
        const size_t nMatches = index.radiusSearch(queryPoint, search_radius, ret_matches, params);
        
        if(nMatches == 1) {
            // outlier
            visited[i] = 1;
            classes[i] = -1;
            types[i] = -1;
            n++;
        } else if (nMatches > k) {
            // core point - start expanding a new cluster
            n++;
            types[i] = 1;
            C = C+1; /* class id of the new cluster */
            visited[i] = 1;
            classes[i] = C;
            matches += nMatches;
            for(int j=0;j<nMatches;j++) {
                size_t idx = ret_matches[j].first;
                if(visited[idx] == 0 && idx!=i) {
                    neigh.push(idx); /* insert neighbors in the neighbors list */
                    visited[idx] = 1;
                    n++;
                }
            }
            // expand cluster until the neighbors list becomes empty
            while(!neigh.empty()) {
                int id = neigh.front();
                
                neigh.pop();
                
                queryPoint[0] = points[id*3];
                queryPoint[1] = points[id*3+1];
                queryPoint[2] = points[id*3+2];
                
                ret_matches.clear();
                ret_matches.reserve(reserveSize);
                
                // find the number of neighbors of current processed neighbor point 
                const size_t nMatches = index.radiusSearch(queryPoint, search_radius, ret_matches, params);
                matches += nMatches;
                if (nMatches < k+1) {
                    // border point
                    types[id] = 0;
                } else {
                    types[id] = 1;
                    for(int j=0;j<nMatches;j++) {
                        size_t idx = ret_matches[j].first;
                        if(visited[idx] == 0 && idx!=id) {
                            neigh.push(idx); /* insert neighbors in the neighbors list */
                            visited[idx] = 1;
                            n++;
                        }
                    }
                }
                
                if(classes[id] == 0)
                    classes[id] = C;
            }
        }
            
     }
  }
  
  free(visited);
}


void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
  double *classes, *types;
  double  k,eps;
  size_t dim,numPoints;
  
  if(nrhs!=3) 
    mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumInputs",
            "Three inputs required.");
  if(nlhs!=2) 
    mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumOutputs",
            "Two output required.");
  
  /* check to make sure the second input argument is a scalar */
  if( !mxIsDouble(prhs[1]) || mxGetN(prhs[1])*mxGetM(prhs[1])!=1 ) {
    mexErrMsgIdAndTxt( "MATLAB:xtimesy:xNotScalar",
            "Input k must be a scalar.");
  }
  
  /* check to make sure the third input argument is a scalar */
  if( !mxIsDouble(prhs[2]) || mxGetN(prhs[2])*mxGetM(prhs[2])!=1 ) {
    mexErrMsgIdAndTxt( "MATLAB:xtimesy:xNotScalar",
            "Input k must be a scalar.");
  }
  
  /*  get the scalar input k */
  k = mxGetScalar(prhs[1]);
  
  /*  get the scalar input eps */
  eps = mxGetScalar(prhs[2]);
  
  /*  get the dimensions of the matrix input x */
  numPoints = mxGetN(prhs[0]);
  dim = mxGetM(prhs[0]);
  
  /*  set the output pointer to the output matrix */
  plhs[0] = mxCreateDoubleMatrix( (mwSize)numPoints, (mwSize)1, mxREAL);
  
  /*  create a C pointer to a copy of the output matrix */
  classes = mxGetPr(plhs[0]);
  
  /*  set the output pointer to the output matrix */
  plhs[1] = mxCreateDoubleMatrix( (mwSize)numPoints, (mwSize)1, mxREAL);
  
  /*  create a C pointer to a copy of the output matrix */
  types = mxGetPr(plhs[1]);
  
  /*  call the C subroutine */
  dbscan(prhs[0],classes,types,k,eps,numPoints,dim);
  
}
