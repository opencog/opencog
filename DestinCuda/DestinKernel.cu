#include "DestinKernel.h"

// C/C++ headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
// Cuda header
#include <cuda.h>
#include <curand.h>

const int AmountThreads = 128;

using namespace std;

__global__ void Destin( int States, int InputDimensionlity, float *dInputData, float *dLayerData, float *dNodeData );
__global__ void sum(int a, int b, int *c);

DestinKernel::DestinKernel( void )
{
    mID=0;
	mRows=0;
	mCols=0;
	mStates=0;
	mInputDimensionlity=0;
	cuDeviceGetCount(&mDevices);
	cout << "Kernel created" << endl;
}

DestinKernel::~DestinKernel( void )
{
    free ( mLayerData ) ;
    cudaFree( dLayerData );
    free ( mNodeData ) ;
    cudaFree( dNodeData );
    cout << "Kernel destroyed" << endl;
}

void DestinKernel::Create( int ID, int Rows, int Cols, int States, int InputDimensionlity)
{
    mID = ID;
    mRows = Rows;
    mCols = Cols;
    mStates = States;
    mInputDimensionlity = InputDimensionlity;

    // Data holder for whole layer including centroids
    // Size of data holder is rows times columns.
    // Cause it needs to hold amount of centroids (mStates) including its vector the whole structure is time centroids and InputDimensionlity.
    sizeOfLayerData = mRows*mCols*mStates*mInputDimensionlity;
    mLayerData = new float[sizeOfLayerData];
    cudaMalloc( (void**)&dLayerData, sizeOfLayerData*sizeof(float) );

    // curandGenerator_t is a CUDA version of rand
    // This fills the whole memory block with number between 0.0 and 1.0
    curandGenerator_t gen;
    curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT);
    // TODO: Put seed code at the place of 1
    curandSetPseudoRandomGeneratorSeed( gen, mID );
    curandGenerateUniform( gen, dLayerData, sizeOfLayerData );

    // TODO: Remove debug line.
    cudaMemcpy ( mLayerData, dLayerData, sizeOfLayerData*sizeof(float), cudaMemcpyDeviceToHost );

    // The generator have to be destroyed after use.
    curandDestroyGenerator( gen );
    // Node data contain the distance to the observation (It's is empty the first run)
    sizeOfNodeData = mRows*mCols*mStates;
    mNodeData = new float[sizeOfNodeData];
    cudaMalloc( (void**)&dNodeData, sizeOfNodeData*sizeof(float) );
}

void DestinKernel::DoDestin( float *Input )
{
    // Threads is the amount of thread inside each block
    dim3 threads( AmountThreads );
    // Grid is the amount of blocks inside a grid
    dim3 grid( mCols, mRows );
    // The launch of the kernel itself with centroids(states), dimension, input data and the Data of the layer itself

    Destin<<<grid, threads, (mInputDimensionlity+mStates)*sizeof(float)>>>( mStates, mInputDimensionlity, Input, dLayerData, dNodeData );

    cudaMemcpy(mNodeData, dNodeData, sizeOfNodeData*sizeof(float), cudaMemcpyDeviceToHost);
    for(int r=0;r<mRows;r++)
    {
        for(int c=0;c<mCols;c++)
        {
            cout << "Node: " << r*mCols+c << endl;
            for(int s=0;s<mStates;s++)
            {
                cout << "Centroid: " << s << " : ";
                cout << mNodeData[r*mCols+c*mStates+s] << endl;
            }
            cout << endl;
        }
    }
}

__global__ void Destin( int States, int InputDimensionlity, float *dInputData, float *dLayerData, float *dNodeData )
{
    // This is how to declare a shared memory inside CUDA.
    extern __shared__ float shared[];
    float* input = (float*)&shared;
    float* distance = (float*)&input[InputDimensionlity*sizeof(float)];

    // We use many threads they need to know where they have to do there work.
    int tid = threadIdx.x;
    int bid = blockIdx.x + blockIdx.y*gridDim.x;

    // make sure the input data is inside shared memory this we are going to compare the amount of centroids.
    while(tid < InputDimensionlity)
    {
        // Put input data for node inside shared memory
        input[tid] = dInputData[tid+bid*InputDimensionlity];
        // A trick for when the dimension is bigger then the amount of threads
        tid += blockDim.x;
    }
    // all threads have to be here to be sure shared memory is filled with the input.
    __syncthreads();

    // calculation distance in massive thread style.
    int node = 0;
    while (node<States)
    {
        tid = threadIdx.x;
        while(tid < InputDimensionlity)
        {
            // Small formula to get to the right working position: dimension*centroids*block+current centroid*dimension+thread
            distance[tid] = input[tid] - dLayerData[InputDimensionlity*States*bid+node*InputDimensionlity+tid];
            distance[tid] = distance[tid] * distance[tid];
            // A trick for when the dimension is bigger then the amount of threads
            tid += blockDim.x;
        }
        // all threads have to wait here so we know all distance have been calculated
        __syncthreads();

        // bit wise divide by 2
        int d = InputDimensionlity >> 1;
        int dOld = d*2;
        tid = threadIdx.x;
        // a sum reduction
        while (d != 0)
        {
            dOld = dOld - d*2;
            while(tid < d)
            {
                distance[tid] += distance[tid + d];
                if (dOld == 1 && tid == d)
                {
                    distance[tid] += distance[tid+d+1];
                }
                tid += blockDim.x;
            }
            // Sync moment before starting with next iteration of reduction.
            __syncthreads();

            dOld = d;
            d >>= 1;
        }
        // Write distance to Node Data
        if(tid == 0)
        {
            dNodeData[node+bid*States] = sqrt(distance[tid]);
        }
        node++;
    }
}
