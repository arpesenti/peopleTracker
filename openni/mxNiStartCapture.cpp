#include "mex.h"
#include "math.h"
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#define START_CAPTURE_CHECK_RC(rc, what) \
if (nRetVal != XN_STATUS_OK)			 \
{										 \
  printf("Failed to %s: %s\n", what, xnGetStatusString(rc)); \
  mexErrMsgTxt("Error");									\
}


//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------

xn::Context g_Context;
xn::ImageGenerator g_Image;
xn::DepthGenerator g_DepthGenerator;
xn::IRGenerator g_IR;
xn::Recorder g_Recorder;


/* The matlab mex function */
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {
    XnUInt64 *MXadress;
    XnUInt64 *MXadressOut;
    
    const mwSize Jdimsc[2]={1, 1};
    int nRetVal;
    char *SAMPLE_DATA_PATH;
    
    if(nrhs<2) {
        printf("Close failed: Give Pointer to Kinect, and filename as input\n");
        mexErrMsgTxt("Kinect Error");
    }
    MXadress = (XnUInt64*)mxGetData(prhs[0]);
    if(MXadress[0]>0){ g_Context = ((xn::Context*) MXadress[0])[0]; }
    
    // Create the output pointer
    plhs[0] = mxCreateNumericArray(2, Jdimsc, mxUINT64_CLASS, mxREAL);
    MXadressOut = (XnUInt64*)mxGetData(plhs[0]);
    
    // Create OpenNI Recorder
    nRetVal = g_Recorder.Create(g_Context);
    START_CAPTURE_CHECK_RC(nRetVal, "Create a Recorder");

    // Connect to file
    SAMPLE_DATA_PATH=mxArrayToString(prhs[1]);
    nRetVal = g_Recorder.SetDestination(XN_RECORD_MEDIUM_FILE, SAMPLE_DATA_PATH);
    
    if(MXadress[1]>0){
        g_Image = ((xn::ImageGenerator*) MXadress[1])[0];
        nRetVal = g_Recorder.AddNodeToRecording(g_Image, XN_CODEC_UNCOMPRESSED);
        START_CAPTURE_CHECK_RC(nRetVal, "Record Image");
    }
    
    if(MXadress[2]>0) {
        g_DepthGenerator = ((xn::DepthGenerator*) MXadress[2])[0];
        nRetVal = g_Recorder.AddNodeToRecording(g_DepthGenerator, XN_CODEC_16Z_EMB_TABLES);
        START_CAPTURE_CHECK_RC(nRetVal, "Record Depth");
    }
    
    if(MXadress[3]>0) {
        g_IR = ((xn::IRGenerator*) MXadress[3])[0];
        nRetVal = g_Recorder.AddNodeToRecording(g_IR, XN_CODEC_UNCOMPRESSED);
        START_CAPTURE_CHECK_RC(nRetVal, "Record IR");
    }
    
    // Return the pointer of the Recorder to Matlab
    MXadressOut[0] = (XnUInt64)&g_Recorder;
}


