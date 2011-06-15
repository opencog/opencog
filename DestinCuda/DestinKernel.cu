#include "DestinKernel.h"

// C/C++ headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
// Cuda header
#include <cuda.h>
#include <curand.h>

using namespace std;

__global__ void Destin( int States, int InputDimensionlity, float *image, float *dLayerData );

DestinKernel::DestinKernel( void )
{
	mRows=0;
	mCols=0;
	mStates=0;
	mInputDimensionlity=0;
	cuDeviceGetCount(&mDevices);
	cout << "Layer created" << endl;
}

DestinKernel::~DestinKernel( void )
{
    free ( mLayerData ) ;
    cudaFree( dLayerData );
    cout << "Layer destroyed" << endl;
}

void DestinKernel::Create( int Rows, int Cols, int States, int InputDimensionlity )
{
    mRows = Rows;
    mCols = Cols;
    mStates = States;
    mInputDimensionlity = InputDimensionlity;

    // Data holder for whole layer including centroids
    // Size of data holder is rows times columns.
    // Cause it needs to hold amount of centroids (mStates) including its vector the whole structre is time centroids and InputDimensionlity.
    int size = mRows*mCols*mStates*mInputDimensionlity;
    mLayerData = (float*) calloc(size, sizeof(float) );
    cudaMalloc( (void**)&dLayerData, size*sizeof(float) );

    // curandGenerator_t is a CUDA version of rand
    // This fills the whole memory block with number between 0.0 and 1.0
    curandGenerator_t gen;
    curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT);
    // TODO: Put seed code at the place of 1
    curandSetPseudoRandomGeneratorSeed( gen, 1 );
    curandGenerateUniform( gen, dLayerData, size );
    // TODO: Remove debug line.
    cudaMemcpy ( mLayerData ,dLayerData , size*sizeof(float), cudaMemcpyDeviceToHost );
    curandDestroyGenerator( gen );
}

void DestinKernel::DoDestin( float *image )
{
    dim3 threads( 64, 1 );
    dim3 grid( mCols, mRows );
    Destin<<<grid,threads>>>( mStates, mInputDimensionlity, image, dLayerData );
}

__global__ void Destin( int States, int InputDimensionlity, float *image, float *dLayerData )
{
    __shared__ float* observation;
    int x,y;
    x = blockDim.x;
    y = blockDim.y;

}
