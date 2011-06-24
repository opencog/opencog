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

__global__ void CalculateDistance( int States, int InputDimensionlity, float *InputData, float *CentroidVectorData, float *CentroidDist, int *WinningCentroids, float *CentroidStarvation, float *Output );
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
    free ( mCentroidsVectorData );
    cudaFree( dCentroidsVectorData );
    free ( mCentroidsDistance );
    cudaFree( dCentroidsDistance );
    free ( mCentroidStarvation );
    cudaFree( dCentroidStarvation );
    free ( mWinningCentroids );
    cudaFree( dWinningCentroids );
    free ( mNodeOutput );
    cudaFree( dNodeOutput );
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
    mCentroidsDistance = new float[sizeOfNodeData];
    cudaMalloc( (void**)&dCentroidsDistance, sizeOfNodeData*sizeof(float) );

    // Starvation data for all centroids
    mCentroidStarvation = new float[sizeOfNodeData];
    cudaMalloc( (void**)&dCentroidStarvation, sizeOfNodeData*sizeof(float) );
    for(int i=0;i<sizeOfNodeData;i++)
    {
        mCentroidStarvation[i]=1.0f;
    }
    // Copy the data from host to device
    cudaMemcpy(dCentroidStarvation, mCentroidStarvation, sizeOfNodeData*sizeof(float), cudaMemcpyHostToDevice);

    // Output for next layer
    mNodeOutput = new float[sizeOfNodeData];
    cudaMalloc( (void**)&dNodeOutput, sizeOfNodeData*sizeof(float) );

    // The layer data is the one that hold all vectors for all centroids inside each layer
    mCentroidsVectorData = new float[sizeOfLayerData];
    cudaMalloc( (void**)&dCentroidsVectorData, sizeOfLayerData*sizeof(float) );
    // This is to fill the dLayerData with all random numbers between 0.0 and 1.0
    curandGenerateUniform( gen, dCentroidsVectorData, sizeOfLayerData );
    // TODO: (Re)move debug line.
    // cudaMemcpy ( mCentroidVectorData, dCentroidVectorData, sizeOfLayerData*sizeof(float), cudaMemcpyDeviceToHost );
}

void DestinKernel::DoDestin( float *Input )
{
    cout << "Layer: " << mID << endl;
    // Threads is the amount of thread inside each. block
    dim3 threads( AmountThreads );
    // Grid is the amount of blocks inside a grid.
    dim3 grid( mCols, mRows );
    // Cause of the use of dynamic shared memory you have to tell the kernel how much shared memory space you need for each block.
    int sharedMem = (mInputDimensionlity+mInputDimensionlity+mStates+mStates+mStates)*sizeof(float);
    // The launch of the kernel itself with centroids(states), dimension, input data and the Data of the layer itself
    CalculateDistance<<<grid, threads, sharedMem>>>( mStates, mInputDimensionlity, Input, dCentroidsVectorData, dCentroidsDistance, dWinningCentroids, dCentroidStarvation, dNodeOutput );
    UpdateStarvation<<<grid, threads>>>( mStates, mSTARVATION_COEFFICIENT, dWinningCentroids, dCentroidStarvation );

    // TODO: move this debug information or make sure the data is ready for other parts of DeSTIN.
    cudaMemcpy(mCentroidsDistance, dCentroidsDistance, sizeOfNodeData*sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(mCentroidStarvation, dCentroidStarvation, sizeOfNodeData*sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(mNodeOutput, dNodeOutput, sizeOfNodeData*sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(mWinningCentroids, dWinningCentroids, sizeOfNodes*sizeof(int), cudaMemcpyDeviceToHost);

    for(int r=0;r<mRows;r++)
    {
        for(int c=0;c<mCols;c++)
        {
            cout << "Node: " << r*mCols+c << endl;
            cout << "Winning: " << mWinningCentroids[r*mCols+c] << endl;
            for(int s=0;s<mStates;s++)
            {
                cout << "Centroid: " << s << " : " << mCentroidsDistance[(c+r*mCols)*mStates+s];
                cout << " Starvation: " << mCentroidStarvation[(c+r*mCols)*mStates+s];
                cout << " OutPut: " << mNodeOutput[(c+r*mCols)*mStates+s];
                cout << endl;
            }
            cout << endl;
        }
    }
}

__global__ void CalculateDistance( int States, int InputDimensionlity, float *InputData, float *CentroidVectorData, float *CentroidDist, int *WinningCentroids, float *CentroidStarvation, float *Output )
{
    // This is how to declare a shared memory inside CUDA.
    extern __shared__ float shared[];
    float* input = (float*)&shared;
    float* distance = (float*)&input[InputDimensionlity];
    float* winner = (float*)&distance[InputDimensionlity];
    float* winnerId = (float*)&winner[States];
    float* tPOS = (float*)&winnerId[States];

    // We use many threads they need to know where they have to do there work.
    // tid (Thread ID) is the amount of threads inside a block its a fixed amount it can be changed by changing: AmountThreads.
    // Keep in mind that CUDA threads should be in steps of 32 (each warp takes 4 clock cycles where each cycle calculate 8 threads)
    int tid = threadIdx.x;
    // bid (Block ID) this keeps track in witch node we are working you can ask the grid the size of the blocks used in x or y and on a Fermi or higher even z
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
    // keep track of the centroid
    int centroid = 0;
    while (centroid<States)
    {
        // reset the tid
        tid = threadIdx.x;
        while(tid < InputDimensionlity)
        {
            // This temp will have for a short while the calculation of input - centroid for position tid (one cell of the vector)
            float temp = 0.0f;
            // distance to input = (input - centroid)*(input - centroid)
            // Small formula to get to the right working position: dimension*centroids*block+current centroid*dimension+thread
            temp = (input[tid] - CentroidVectorData[InputDimensionlity*States*bid+centroid*InputDimensionlity+tid]);
            distance[tid] = temp * temp;
            // A trick for when the dimension is bigger then the amount of threads
            tid += blockDim.x;
        }
        // all threads have to wait here so we know all distance have been calculated
        __syncthreads();

        // bite wise divide by 2 (should be faster the /2)
        int d = InputDimensionlity >> 1;
        // Cause DeSTIN don't work with numbers that are 2^? we have to check for odd numbers
        int dOld = d*2;
        // a sum reduction, This is a common trick on CUDA to add shared memory instead of striding true memory
        // You have to use half the memory each step and each thread will add itself to with the other half.
        while (d != 0)
        {
            // reset the tid
            tid = threadIdx.x;
            dOld = dOld - d*2;
            while(tid < d)
            {
                // the adding calculation
                distance[tid] += distance[tid + d];

                // special case in case of odd number (As long as this don't happen to often it won't effect speed)
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
            // square root on sum of the (input - centroid)*(input - centroid)
            dist = (sqrt(distance[tid]))*CentroidStarvation[centroid+bid*States];
            // Index of centroid inside node
            winnerId[centroid] = centroid;
            // Fill shared memory with distance of each centroid
            winner[centroid] = dist;
            // POS calculation from original DeSTIN
            tPOS[centroid] = (float)(1.0/(1e-9+(double)dist));
            // For debugging or analyzing saving the Distance to the observation
            // (Remember that you should copy the data from the device to the host and store it then)
            CentroidDist[centroid+bid*States] = dist;
        }
        // go to next centroid inside the node (bid is taking care of the other node)
        centroid++;
    }
    __syncthreads();

    // Reduction trick again to find winning centroid
    // (Looks like merge sort only instead of sorting everything just move the winning centroid to position 0)
    // The sum of tPOS will be done also cause we need that one later
    int d = States >> 1;
    int dOld = d*2;
    while (d != 0)
    {
        tid = threadIdx.x;
        dOld = dOld - d*2;
        while(tid < d)
        {
            // Adding tPOS
            tPOS[tid] += tPOS[tid + d];
            if(winner[tid] > winner[tid + d])
            {
                // Move winning centroid to the beginning
                winnerId[tid] = winnerId[tid + d];
            }

            if (dOld == 1 && tid == d-1)
            {
                // special case of odd numbers
                tPOS[tid] += tPOS[tid + d + 1];
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
    while(tid < States)
    {
        // This is the POS for all centroids (It looks like this is the input for the next layer also)
        // The output is missing the advice of higher layer
        Output[tid+bid*States] = (float)(1.0/(1e-9+(double)CentroidDist[tid+bid*States]))/tPOS[0];
        tid += blockDim.x;
    }
}

__global__ void UpdateStarvation( int States, float StarvationCoefficient, int *WinningCentroids, float *CentroidStarvation )
{
    // This is the updating starvation fast and quick to update all the nodes and reset the winning centroid
    // According to DeSTIN paper: The winning centroid starvation gets reset while the others starve more
    // Aldo this is the simple version of it it might be changed in the further cause this make the network also forget what it learn
    // when it is looking at something else for a very long time (Short and Long term memory)
    // for tid and bid see CalculateDistance kernel.
    int tid = threadIdx.x;
    int bid = blockIdx.x + blockIdx.y * gridDim.x;
    while(tid < States)
    {
        // Let all centroid starve
        CentroidStarvation[tid+bid*States] = (1.0f-StarvationCoefficient)*CentroidStarvation[tid+bid*States];
        // Reset winning centroid
        CentroidStarvation[WinningCentroids[bid]+bid*States] = 1.0f;
        tid += blockDim.x;
    }
}
