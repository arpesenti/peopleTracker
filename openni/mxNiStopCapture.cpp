#include "mex.h"
#include "math.h"
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
    
//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::Recorder g_Recorder;

/* The matlab mex function */
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) {
    XnUInt64 *MXadress;
    if(nrhs==0)
    {
       printf("Close failed: Give Pointer to Recorder object as input\n");
       mexErrMsgTxt("Kinect Error"); 
    }
    MXadress = (XnUInt64*)mxGetData(prhs[0]);
    if(MXadress[0]>0){ g_Recorder = ((xn::Recorder*) MXadress[0])[0]; }
    g_Recorder.Release();
}
