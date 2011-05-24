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

DestinKernel::DestinKernel(void)
{
	mRows=0;
	mCols=0;
	mStates=0;
	cuDeviceGetCount(&mDevices);
}

DestinKernel::~DestinKernel(void)
{
}

void DestinKernel::Create( int Rows, int Cols, int States)
{
    mRows = Rows;
    mCols = Cols;
    mStates = States;
}

__global__ void Layer()
{

}
