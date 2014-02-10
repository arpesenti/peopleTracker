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
xn::ImageGenerator g_Image;

/* The matlab mex function */
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
    char *Iout;
    XnUInt64 *MXadress;
    
    if(nrhs==0)
    {
       printf("Open failed: Give Pointer to Kinect as input\n");
       mexErrMsgTxt("Kinect Error"); 
    }
       
    MXadress = (XnUInt64*)mxGetData(prhs[0]);
    if(MXadress[0]>0){ g_Context = ((xn::Context*) MXadress[0])[0]; }
	if(MXadress[1]>0){ 
		 g_Image = ((xn::ImageGenerator*) 
			 MXadress[1])[0]; 
	}
	else
	{
		mexErrMsgTxt("No Image Node in Kinect Context"); 
	}
    
    XnStatus nRetVal = XN_STATUS_OK;
 	xn::ImageMetaData imageMD;
    
	// Process the data
	g_Image.GetMetaData(imageMD);
    
    // RGB is the only image format supported.
	if (imageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24)
	{
		printf("The device image format must be RGB24\n");
		mexErrMsgTxt("Kinect Error"); 
	}
    
  	XnUInt16 g_nXRes = imageMD.FullXRes();
	XnUInt16 g_nYRes = imageMD.FullYRes();
	const XnUInt8* pImage = imageMD.Data();
    const mwSize Jdimsc[3] = {3, g_nXRes, g_nYRes};
    
    plhs[0] = mxCreateNumericArray(3, Jdimsc, mxUINT8_CLASS, mxREAL);
    Iout = (char*)mxGetData(plhs[0]);
    memcpy (Iout,pImage,Jdimsc[0]*Jdimsc[1]*Jdimsc[2]);  
}
