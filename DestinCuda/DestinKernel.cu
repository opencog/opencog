#include "DestinKernel.h"

// C/C++ headers
#include <stdio.h>
#include <math.h>
#include <vector>
#include <exception>
#include <stdexcept>
// Cuda header
#include <cuda.h>

using namespace std;

__global__ void Layer( int States, int InputDimensionlity, float *image, float *dLayerData );

DestinKernel::DestinKernel( void )
{
	mRows=0;
	mCols=0;
	mStates=0;
	mInputDimensionlity=0;
	cuDeviceGetCount(&mDevices);
}

DestinKernel::~DestinKernel( void )
{
    cudaFree(dLayerData);
}

void DestinKernel::Create( int Rows, int Cols, int States, int InputDimensionlity )
{
    mRows = Rows;
    mCols = Cols;
    mStates = States;
    mInputDimensionlity = InputDimensionlity;
}

void DestinKernel::DoDestin( float *image )
{
    int size = mRows*mCols*mStates;
    cudaMalloc( (void**)&dLayerData, size*sizeof(float) );

    dim3 threads(64, 1);
    dim3 grid(mCols, mRows);
    Layer<<<threads,grid>>>(mStates, mInputDimensionlity, image, dLayerData);
}

__global__ void Layer( int States, int InputDimensionlity, float *image, float *dLayerData )
{
    __shared__ float* observation;
    threadIdx.x;
    blockIdx.x;
}
