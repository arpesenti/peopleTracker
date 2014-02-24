MATLABDIR ?= /Applications/MATLAB_R2013a.app
OPENNIDIR ?= /usr/local/include/ni

CXX = g++
CFLAGS = -O3 -fPIC -fopenmp -I$(MATLABDIR)/extern/include 

LD_LIBRARY=/usr/lib:/usr/lib64:$(LD_LIBRARY_PATH)

LDFLAGS+="-fopenmp"
ifeq ($(MEX_EXT),mexmaci64)
	LDFLAGS+=" -bundle"
else
	LDFLAGS+=" -shared"
endif

ifeq ($(wildcard $(MATLABDIR)/bin/mexext),)
	MEX_EXT = 
else
	MEX_EXT = $(shell $(MATLABDIR)/bin/mexext)
endif

MEX = $(MATLABDIR)/bin/mex
MEX_OPTION = CC\=$(CXX) CXX\=$(CXX) CFLAGS\="$(CFLAGS) -fpermissive" CXXFLAGS\="$(CFLAGS) -std=c++11" LDFLAGS\=$(LDFLAGS) LD_LIBRARY_PATH\=$(LD_LIBRARY)
# comment the following line if you use MATLAB on 32-bit computer
MEX_OPTION += -largeArrayDims

OPENNI_LIB = -lOpenNI

all: matlab libsvm openni

matlab:	expandCandidates.$(MEX_EXT) getVoxelGrid.$(MEX_EXT) dbscanClustering.$(MEX_EXT) extractFeaturesHOG.$(MEX_EXT) minDist.$(MEX_EXT) mMD5.$(MEX_EXT)

libsvm: libsvm/libsvmpredict.$(MEX_EXT) libsvm/libsvmtrain.$(MEX_EXT) libsvm/libsvmreadmodel.$(MEX_EXT) libsvm/libsvmpredictfast.$(MEX_EXT)

openni: openni/mxNiCreateContext.$(MEX_EXT) openni/mxNiDeleteContext.$(MEX_EXT) openni/mxNiDepth.$(MEX_EXT) openni/mxNiPhoto.$(MEX_EXT) openni/mxNiStartCapture.$(MEX_EXT) openni/mxNiStartCapture.$(MEX_EXT) openni/mxNiStopCapture.$(MEX_EXT) openni/mxNiUpdateContext.$(MEX_EXT)

openni/mxNiCreateContext.$(MEX_EXT): openni/mxNiCreateContext.cpp
	$(MEX) $(MEX_OPTION) -I$(OPENNIDIR) $(OPENNI_LIB) -output openni/mxNiCreateContext.$(MEX_EXT) openni/mxNiCreateContext.cpp

openni/mxNiDeleteContext.$(MEX_EXT): openni/mxNiDeleteContext.cpp
	$(MEX) $(MEX_OPTION) -I$(OPENNIDIR) $(OPENNI_LIB) -output openni/mxNiDeleteContext.$(MEX_EXT) openni/mxNiDeleteContext.cpp

openni/mxNiDepth.$(MEX_EXT): openni/mxNiDepth.cpp
	$(MEX) $(MEX_OPTION) -I$(OPENNIDIR) $(OPENNI_LIB) -output openni/mxNiDepth.$(MEX_EXT) openni/mxNiDepth.cpp

openni/mxNiPhoto.$(MEX_EXT): openni/mxNiPhoto.cpp
	$(MEX) $(MEX_OPTION) -I$(OPENNIDIR) $(OPENNI_LIB) -output openni/mxNiPhoto.$(MEX_EXT) openni/mxNiPhoto.cpp

openni/mxNiStartCapture.$(MEX_EXT): openni/mxNiStartCapture.cpp
	$(MEX) $(MEX_OPTION) -I$(OPENNIDIR) $(OPENNI_LIB) -output openni/mxNiStartCapture.$(MEX_EXT) openni/mxNiStartCapture.cpp

openni/mxNiStopCapture.$(MEX_EXT): openni/mxNiStopCapture.cpp
	$(MEX) $(MEX_OPTION) -I$(OPENNIDIR) $(OPENNI_LIB) -output openni/mxNiStopCapture.$(MEX_EXT) openni/mxNiStopCapture.cpp

openni/mxNiUpdateContext.$(MEX_EXT): openni/mxNiUpdateContext.cpp
	$(MEX) $(MEX_OPTION) -I$(OPENNIDIR) $(OPENNI_LIB) -output openni/mxNiUpdateContext.$(MEX_EXT) openni/mxNiUpdateContext.cpp

libsvm/libsvmpredict.$(MEX_EXT):     libsvm/libsvmpredict.c libsvm/svm.h libsvm/svm.o libsvm/svm_model_matlab.o
	$(MEX) $(MEX_OPTION) -output libsvm/libsvmpredict.$(MEX_EXT) libsvm/libsvmpredict.c libsvm/svm.o libsvm/svm_model_matlab.o

libsvm/libsvmpredictfast.$(MEX_EXT):     libsvm/libsvmpredictfast.c libsvm/svm.h libsvm/svm.o libsvm/svm_model_matlab.o
	$(MEX) $(MEX_OPTION) -output libsvm/libsvmpredictfast.$(MEX_EXT) libsvm/libsvmpredictfast.c libsvm/svm.o libsvm/svm_model_matlab.o

libsvm/libsvmtrain.$(MEX_EXT):       libsvm/libsvmtrain.c libsvm/svm.h libsvm/svm.o libsvm/svm_model_matlab.o
	$(MEX) $(MEX_OPTION) -output libsvm/libsvmtrain.$(MEX_EXT) libsvm/libsvmtrain.c libsvm/svm.o libsvm/svm_model_matlab.o

libsvm/libsvmreadmodel.$(MEX_EXT):	libsvm/libsvmreadmodel.c libsvm/svm_model_matlab.o
	$(MEX) $(MEX_OPTION) -output libsvm/libsvmreadmodel.$(MEX_EXT) libsvm/libsvmreadmodel.c libsvm/svm.o libsvm/svm_model_matlab.o

libsvm/svm_model_matlab.o:     libsvm/svm_model_matlab.c libsvm/svm.h
	$(CXX) $(CFLAGS) -c libsvm/svm_model_matlab.c -o libsvm/svm_model_matlab.o

libsvm/svm.o: libsvm/svm.cpp libsvm/svm.h
	make -C libsvm svm.o

expandCandidates.$(MEX_EXT):     expandCandidates.cpp nanoflann.hpp
	$(MEX) $(MEX_OPTION) expandCandidates.cpp

getVoxelGrid.$(MEX_EXT): getVoxelGrid.cpp
	$(MEX) $(MEX_OPTION) getVoxelGrid.cpp

dbscanClustering.$(MEX_EXT): dbscanClustering.cpp nanoflann.hpp
	$(MEX) $(MEX_OPTION) dbscanClustering.cpp

extractFeaturesHOG.$(MEX_EXT): extractFeaturesHOG.cpp
	$(MEX) $(MEX_OPTION) extractFeaturesHOG.cpp

minDist.$(MEX_EXT): minDist.cpp
	$(MEX) $(MEX_OPTION) minDist.cpp

mMD5.$(MEX_EXT): mMD5.c
	$(MEX) $(MEX_OPTION) mMD5.c

clean:
	rm -f *~ *.o *.mex* *.obj libsvm/*.o libsvm/*.mex* openni/*.mex*

