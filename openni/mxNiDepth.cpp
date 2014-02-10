#include "mex.h"
#include "math.h"
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
        mexErrMsgTxt("Kinect Error"); 							    \
	}
    
//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::Context g_Context;
xn::DepthGenerator g_DepthGenerator;

/* The matlab mex function */
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
    unsigned short *Iout;
    XnUInt64 *MXadress;
    
    if(nrhs==0)
    {
       printf("Open failed: Give Pointer to Kinect as input\n");
       mexErrMsgTxt("Kinect Error"); 
    }
    
    MXadress = (XnUInt64*)mxGetData(prhs[0]);
    if(MXadress[0]>0){ g_Context = ((xn::Context*) MXadress[0])[0]; }
    if(MXadress[2]>0)
	{ 
		g_DepthGenerator = ((xn::DepthGenerator*) MXadress[2])[0]; 
	}
	else
	{
		mexErrMsgTxt("No Depth Node in Kinect Context"); 
	}
    
    XnStatus nRetVal = XN_STATUS_OK;

	xn::DepthMetaData depthMD;
    
	// Process the data
	g_DepthGenerator.GetMetaData(depthMD);
        
  	XnUInt16 g_nXRes = depthMD.FullXRes();
	XnUInt16 g_nYRes = depthMD.FullYRes();
	const XnDepthPixel* pDepth = depthMD.Data();
    const mwSize Jdimsc[2] = {g_nXRes, g_nYRes};
    
    plhs[0] = mxCreateNumericArray(2, Jdimsc, mxUINT16_CLASS, mxREAL);
    Iout = (unsigned short*)mxGetData(plhs[0]);
    memcpy (Iout,pDepth,Jdimsc[0]*Jdimsc[1]*2);  
}
