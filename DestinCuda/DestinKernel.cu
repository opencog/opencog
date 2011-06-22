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

__global__ void CalculateDistance( int States, int InputDimensionlity, float *InputData, float *CentroidVectorData, float *CentroidData, int *WinningCentroids, float *CentroidStarvation );
__global__ void UpdateStarvation( int States, float StarvationCoefficient, int *WinningCentroids, float *CentroidStarvation );

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
    free ( mCentroidVectorData ) ;
    cudaFree( dCentroidVectorData );
    free ( mCentroidData ) ;
    cudaFree( dCentroidData );
    free ( mCentroidStarvation ) ;
    cudaFree( dCentroidStarvation );
    free ( mWinningCentroids ) ;
    cudaFree( dWinningCentroids );
    cout << "Kernel destroyed" << endl;
}

void DestinKernel::Create( int ID, int Rows, int Cols, int States, int InputDimensionlity, curandGenerator_t gen)
{
    mID = ID;
    mRows = Rows;
    mCols = Cols;
    mStates = States;
    mInputDimensionlity = InputDimensionlity;

    mSTARVATION_COEFFICIENT = 1.0/((float)InputDimensionlity*(float)InputDimensionlity);
    if ( mSTARVATION_COEFFICIENT < 1.0/512.0 )
    {
        mSTARVATION_COEFFICIENT=1.0/512.0;
    }

    // Define the data sizes
    // Size of de nodes is rows times columns
    sizeOfNodes = mRows*mCols;
    // Size of the data of nodes is rows times columns times centroids
    sizeOfNodeData = sizeOfNodes*mStates;
    // Size of the layer with all vectors is rows times columns times centroids times input vector
    sizeOfLayerData = sizeOfNodeData*mInputDimensionlity;

    // Array full with all the winning centroids of each node
    mWinningCentroids = new int[sizeOfNodes];
    cudaMalloc( (void**)&dWinningCentroids, sizeOfNodes*sizeof(int) );

    // Node data contain the distance to the observation of all centroids (It's is empty the first run)
    mCentroidData = new float[sizeOfNodeData];
    cudaMalloc( (void**)&dCentroidData, sizeOfNodeData*sizeof(float) );

    // Starvation data for all centroids
    mCentroidStarvation = new float[sizeOfNodeData];
    cudaMalloc( (void**)&dCentroidStarvation, sizeOfNodeData*sizeof(float) );
    for(int i=0;i<sizeOfNodeData;i++)
    {
        mCentroidStarvation[i]=1.0f;
    }
    cudaMemcpy(dCentroidStarvation, mCentroidStarvation, sizeOfNodeData*sizeof(float), cudaMemcpyHostToDevice);

    // The layer data is the one that hold all vectors for all centroids inside each layer
    mCentroidVectorData = new float[sizeOfLayerData];
    cudaMalloc( (void**)&dCentroidVectorData, sizeOfLayerData*sizeof(float) );
    // This is to fill the dLayerData with all random numbers between 0.0 and 1.0
    curandGenerateUniform( gen, dCentroidVectorData, sizeOfLayerData );
    // TODO: (Re)move debug line.
    // cudaMemcpy ( mCentroidVectorData, dCentroidVectorData, sizeOfLayerData*sizeof(float), cudaMemcpyDeviceToHost );
}

void DestinKernel::DoDestin( float *Input )
{
    // Threads is the amount of thread inside each. block
    dim3 threads( AmountThreads );
    // Grid is the amount of blocks inside a grid.
    dim3 grid( mCols, mRows );
    // Cause of the use of dynamic shared memory you have to tell the kernel how much shared memory space you need for each block.
    int sharedMem = (mInputDimensionlity+mInputDimensionlity+mStates+mStates)*sizeof(float);
    // The launch of the kernel itself with centroids(states), dimension, input data and the Data of the layer itself
    CalculateDistance<<<grid, threads, sharedMem>>>( mStates, mInputDimensionlity, Input, dCentroidVectorData, dCentroidData, dWinningCentroids, dCentroidStarvation );
    cout << "Meuk: " << mSTARVATION_COEFFICIENT << endl;
    UpdateStarvation<<<grid, threads>>>( mStates, mSTARVATION_COEFFICIENT, dWinningCentroids, dCentroidStarvation );
    cudaMemcpy(mCentroidData, dCentroidData, sizeOfNodeData*sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(mCentroidStarvation, dCentroidStarvation, sizeOfNodeData*sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(mWinningCentroids, dWinningCentroids, sizeOfNodes*sizeof(int), cudaMemcpyDeviceToHost);
    for(int r=0;r<mRows;r++)
    {
        for(int c=0;c<mCols;c++)
        {
            cout << "Node: " << r*mCols+c << endl;
            cout << "Winning: " << mWinningCentroids[r*mCols+c] << endl;
            for(int s=0;s<mStates;s++)
            {
                cout << "Centroid: " << s << " : ";
                cout << mCentroidData[(c+r*mCols)*mStates+s];
                cout << " Starvation: " << mCentroidStarvation[(c+r*mCols)*mStates+s];
                cout << endl;
            }
            cout << endl;
        }
    }
}

__global__ void CalculateDistance( int States, int InputDimensionlity, float *InputData, float *CentroidVectorData, float *CentroidData, int *WinningCentroids, float *CentroidStarvation )
{
    // This is how to declare a shared memory inside CUDA.
    extern __shared__ float shared[];
    float* input = (float*)&shared;
    float* distance = (float*)&input[InputDimensionlity];
    float* winner = (float*)&distance[InputDimensionlity];
    float* winnerId = (float*)&winner[States];

    // We use many threads they need to know where they have to do there work.
    int tid = threadIdx.x;
    int bid = blockIdx.x + blockIdx.y * gridDim.x;

    // make sure the input data is inside shared memory this we are going to compare the amount of centroids.
    while(tid < InputDimensionlity)
    {
        // Put input data for node inside shared memory
        input[tid] = InputData[tid + bid * InputDimensionlity];
        // A trick for when the dimension is bigger then the amount of threads
        tid += blockDim.x;
    }
    // all threads have to be here to be sure shared memory is filled with the input.
    __syncthreads();

    // calculation distance in massive thread style.
    int node = 0;
    float temp = 0.0f;
    while (node<States)
    {
        tid = threadIdx.x;
        while(tid < InputDimensionlity)
        {
            // Small formula to get to the right working position: dimension*centroids*block+current centroid*dimension+thread
            temp = (input[tid] - CentroidVectorData[InputDimensionlity*States*bid+node*InputDimensionlity+tid]);
            distance[tid] = temp * temp;
            // A trick for when the dimension is bigger then the amount of threads
            tid += blockDim.x;
        }
        // all threads have to wait here so we know all distance have been calculated
        __syncthreads();

        // bit wise divide by 2
        int d = InputDimensionlity >> 1;
        int dOld = d*2;
        // a sum reduction
        while (d != 0)
        {
            tid = threadIdx.x;
            dOld = dOld - d*2;
            while(tid < d)
            {
                distance[tid] += distance[tid + d];

                if (dOld == 1 && tid == d-1)
                {
                    distance[tid] += distance[tid + d + 1];
                }
                tid += blockDim.x;
            }
            // Sync moment before starting with next iteration of reduction.
            __syncthreads();

            dOld = d;
            d >>= 1;
        }

        // Write distance to Node Data
        tid = threadIdx.x;
        float dist = 0.0f;
        if(tid == 0)
        {
            dist = (sqrt(distance[tid]))*CentroidStarvation[node+bid*States];
            winner[node] = dist;
            winnerId[node] = node;
            CentroidData[node+bid*States] = dist;
        }
        node++;
    }
    __syncthreads();

    int d = States >> 1;
    int dOld = d*2;
    // Reduction trick to find winning centroid
    while (d != 0)
    {
        tid = threadIdx.x;
        dOld = dOld - d*2;
        while(tid < d)
        {
            if(winner[tid] > winner[tid + d])
            {
                winnerId[tid] = winnerId[tid + d];
            }

            if (dOld == 1 && tid == d-1)
            {
                if(winner[tid] > winner[tid + d + 1])
                {
                    winnerId[tid] = winnerId[tid + d + 1];
                }
            }
            tid += blockDim.x;
        }
        // Sync moment before starting with next iteration of reduction.
        __syncthreads();

        dOld = d;
        d >>= 1;
    }
    // Write the winning centroid into there position
    tid = threadIdx.x;
    if(tid == 0)
    {
        WinningCentroids[bid] = winnerId[tid];
    }
}

__global__ void UpdateStarvation( int States, float StarvationCoefficient, int *WinningCentroids, float *CentroidStarvation )
{
    int tid = threadIdx.x;
    int bid = blockIdx.x + blockIdx.y * gridDim.x;
    while(tid < States)
    {
        CentroidStarvation[tid+bid*States] = (1.0f-StarvationCoefficient)*CentroidStarvation[tid+bid*States];
        CentroidStarvation[WinningCentroids[bid]+bid*States] = 1.0f;
        tid += blockDim.x;
    }
}
