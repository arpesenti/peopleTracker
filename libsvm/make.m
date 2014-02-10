% This make.m is for MATLAB under Windows, Mac, and Unix

try
    mex MEX_OPTION="\$MEX_OPTION -lgomp" CFLAGS="\$CFLAGS -fopenmp -std=c99" CXXFLAGS="\$CXXFLAGS -fopenmp" LDFLAGS="\$LDFLAGS -fopenmp" -largeArrayDims libsvmpredictfast.c svm.cpp svm_model_matlab.c
    mex MEX_OPTION="\$MEX_OPTION -lgomp" CFLAGS="\$CFLAGS -fopenmp -std=c99" CXXFLAGS="\$CXXFLAGS -fopenmp" LDFLAGS="\$LDFLAGS -fopenmp" -largeArrayDims libsvmtrain.c svm.cpp svm_model_matlab.c
    mex MEX_OPTION="\$MEX_OPTION -lgomp" CFLAGS="\$CFLAGS -fopenmp -std=c99" CXXFLAGS="\$CXXFLAGS -fopenmp" LDFLAGS="\$LDFLAGS -fopenmp" -largeArrayDims libsvmpredict.c svm.cpp svm_model_matlab.c
    mex MEX_OPTION="\$MEX_OPTION -lgomp" CFLAGS="\$CFLAGS -fopenmp -std=c99" CXXFLAGS="\$CXXFLAGS -fopenmp" LDFLAGS="\$LDFLAGS -fopenmp" -largeArrayDims libsvmreadmodel.c svm.cpp svm_model_matlab.c
catch
	fprintf('If make.m fails, please check README about detailed instructions.\n');
end
