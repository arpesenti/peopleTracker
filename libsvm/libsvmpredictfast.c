#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "svm.h"
#include <sys/time.h>
#include <stdint.h>


#include "mex.h"
#include "svm_model_matlab.h"

#ifdef MX_API_VER
#if MX_API_VER < 0x07030000
typedef int mwIndex;
#endif
#endif

#define CMD_LEN 2048

int print_null(const char *s,...) {}
int (*info)(const char *fmt,...) = &mexPrintf;

static void fake_answer(int nlhs, mxArray *plhs[])
{
	int i;
	for(i=0;i<nlhs;i++)
		plhs[i] = mxCreateDoubleMatrix(0, 0, mxREAL);
}

void predict(int nlhs, mxArray *plhs[], const mxArray *prhs[], struct svm_model *model)
{
	int feature_number, testing_instance_number;
	int instance_index;
	double *ptr_instance; 
	double *ptr_prob_estimates;
	
	mxArray *tplhs; // temporary storage for plhs[]

	int svm_type=svm_get_svm_type(model);
	int nr_class=svm_get_nr_class(model);

	// prhs[0] = testing instance matrix
	feature_number = (int)mxGetM(prhs[0]);
	testing_instance_number = (int)mxGetN(prhs[0]);

	ptr_instance = mxGetPr(prhs[0]);

	tplhs = mxCreateDoubleMatrix(testing_instance_number, nr_class, mxREAL);

	ptr_prob_estimates = mxGetPr(tplhs);    
    
    int freeInstance = 0;
    
	#pragma omp parallel
    {
    struct svm_node *x = (struct svm_node*)malloc((feature_number+1)*sizeof(struct svm_node) );
    double *prob_estimates = (double *) malloc(nr_class*sizeof(double));
    int i,base;
    double predict_label; 
    
    int instance_index;
    while (true) {
        #pragma omp critical 
        {
          instance_index = freeInstance;
          freeInstance++;
        }

        if (instance_index >= testing_instance_number)
            break;    
        
        base = feature_number*instance_index;
        for(i=0;i<feature_number;i++)
        {
            x[i].index = i+1;
            x[i].value = ptr_instance[base + i];
        }
        x[feature_number].index = -1;
        
        predict_label = svm_predict_probability(model, x, prob_estimates);

        for(i=0;i<nr_class;i++)
            ptr_prob_estimates[instance_index + i * testing_instance_number] = prob_estimates[i];
    }
    free(x);
    free(prob_estimates);
    }
		
    plhs[0] = tplhs;

}

void exit_with_help()
{
	mexPrintf(
		"Usage: [predicted_label, accuracy, decision_values/prob_estimates] = svmpredict(testing_label_vector, testing_instance_matrix, model, 'libsvm_options')\n"
		"       [predicted_label] = svmpredict(testing_label_vector, testing_instance_matrix, model, 'libsvm_options')\n"
		"Parameters:\n"
		"  model: SVM model structure from svmtrain.\n"
		"  libsvm_options:\n"
		"    -b probability_estimates: whether to predict probability estimates, 0 or 1 (default 0); one-class SVM not supported yet\n"
		"    -q : quiet mode (no outputs)\n"
		"Returns:\n"
		"  predicted_label: SVM prediction output vector.\n"
		"  accuracy: a vector with accuracy, mean squared error, squared correlation coefficient.\n"
		"  prob_estimates: If selected, probability estimate vector.\n"
	);
}

void mexFunction( int nlhs, mxArray *plhs[],
		 int nrhs, const mxArray *prhs[] )
{
	struct svm_model *model;
	info = &mexPrintf;

	if(nlhs > 2 || nrhs != 2)
	{
		exit_with_help();
		fake_answer(nlhs, plhs);
		return;
	}

	if(!mxIsDouble(prhs[0])) {
		mexPrintf("Error: label vector and instance matrix must be double\n");
		fake_answer(nlhs, plhs);
		return;
	}

    uint64_t *address = (uint64_t *)mxGetPr(prhs[1]);
    model = (struct svm_model *)(*address);
        
//     struct timeval start,stop;
//     gettimeofday(&start,NULL);

    predict(nlhs, plhs, prhs, model);

//     gettimeofday(&stop,NULL);
//     mexPrintf("%lu\n",stop.tv_usec - start.tv_usec);
    
    plhs[1] = mxCreateDoubleMatrix(1, model->nr_class, mxREAL);
    double *ptr = mxGetPr(plhs[1]);
    for(int i = 0; i < model->nr_class; i++)
			ptr[i] = model->label[i];

	return;
}
