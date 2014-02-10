#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "svm.h"

#include "mex.h"
#include "svm_model_matlab.h"

void mexFunction( int nlhs, mxArray *plhs[],
		 int nrhs, const mxArray *prhs[] )
{
	struct svm_model *model;
	
	if(nrhs != 1 || !mxIsStruct(prhs[0])) {
		mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumInputs",
            "Input has to be a svm struct.");
	}
    
    const char *error_msg;

    model = matlab_matrix_to_model(prhs[0], &error_msg);
    
    if (model == NULL)
    {
        mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumInputs",
            "Error: can't read model");
    }

    if(svm_check_probability_model(model)==0)
    {
        mexErrMsgIdAndTxt( "MATLAB:xtimesy:invalidNumInputs",
            "Model does not support probabiliy estimates");
    }
    
    mwSize dim[]={1,1};
    plhs[0] = mxCreateNumericArray(2, dim, mxUINT64_CLASS, mxREAL);
	uint64_t *address = (uint64_t *)mxGetData(plhs[0]);
    *address = (uint64_t)model;
    
}
