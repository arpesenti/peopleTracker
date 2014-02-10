#include "mex.h"
#include "math.h"
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
		return;												\
	}
    
//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::Context g_Context;
xn::IRGenerator g_ir;
xn::ImageGenerator g_image;
xn::UserGenerator g_UserGenerator;
xn::DepthGenerator g_DepthGenerator;

/* The matlab mex function */
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {
    XnUInt64 *MXadress;
    XnUInt64 address;
    int argc=0;
    const mwSize Jdimsc[2]={1,5};
    char *SAMPLE_XML_PATH;
    char *SAMPLE_DATA_PATH;
    
    // Output the Point to the Kinect Object
    plhs[0] = mxCreateNumericArray(2, Jdimsc, mxUINT64_CLASS, mxREAL);
  
    if(nrhs==0)
    {
       printf("Open failed: No XML path given \n");
       mexErrMsgTxt("Kinect Error"); 
    }
    
    XnStatus nRetVal = XN_STATUS_OK;
    if (nrhs > 1) {
         SAMPLE_DATA_PATH=mxArrayToString(prhs[1]);
         nRetVal = g_Context.Init();
         CHECK_RC(nRetVal, "Init");
         nRetVal = g_Context.OpenFileRecording(SAMPLE_DATA_PATH);
         if (nRetVal != XN_STATUS_OK) {
            printf("Can't open recording %s: %s\n", SAMPLE_DATA_PATH, xnGetStatusString(nRetVal));
            mexErrMsgTxt("Kinect Error"); 
         }
         else
         {
            address=( XnUInt64)&g_Context;
         }
    }
    else {
        SAMPLE_XML_PATH=mxArrayToString(prhs[0]);
        xn::EnumerationErrors errors;
        nRetVal = g_Context.InitFromXmlFile(SAMPLE_XML_PATH, &errors);
        if (nRetVal == XN_STATUS_NO_NODE_PRESENT) {
            XnChar strError[1024];
            errors.ToString(strError, 1024);
            printf("%s\n", strError);
            mexErrMsgTxt("Kinect Error"); 
        }
        else if (nRetVal != XN_STATUS_OK) {
            printf("Open failed: %s\n", xnGetStatusString(nRetVal));
            mexErrMsgTxt("Kinect Error"); 
        }
        else
        {
            address=( XnUInt64)&g_Context;
        }
    }
    
    MXadress = (XnUInt64*)mxGetData(plhs[0]);
    MXadress[0] = address;
    
    // Detect Image
    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);
	if (nRetVal != XN_STATUS_OK)								
	{	
        printf("Image Node : Not found\n");
        MXadress[1] = 0;
    }
    else
    {
        printf("Image Node : Found\n");
        MXadress[1] = (XnUInt64)&g_image;
    }
    
    // Detect Depth
    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	if (nRetVal != XN_STATUS_OK)								
	{	
        printf("Depth Node : Not found\n");
        MXadress[2] = 0;
    }
    else
    {
        printf("Depth Node : Found\n");
        MXadress[2] = (XnUInt64)&g_DepthGenerator;
    }
    
    // Detect Infrared
    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_IR, g_ir);
	if (nRetVal != XN_STATUS_OK)								
	{	
        printf("Infrared Node :Not found\n");
        MXadress[3] = 0;
    }
    else
    {
        printf("Infrared Node : Found\n");
        MXadress[3] = (XnUInt64)&g_ir;
    }
    
    
    MXadress[4] = 0;
    
//     nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
// 	if (nRetVal != XN_STATUS_OK)
// 	{
// 		nRetVal = g_UserGenerator.Create(g_Context);
// 	}
//         
//     if (nRetVal != XN_STATUS_OK)								
//     {	
//          printf("User Node : Not found\n");
//          MXadress[4] = 0;
//     }
//     else
//     {
//         printf("User Node : Found\n");
//         MXadress[4] = (XnUInt64)&g_UserGenerator;
//     }
    
            
	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");
    // Read next available data
    g_Context.WaitAndUpdateAll();
}
