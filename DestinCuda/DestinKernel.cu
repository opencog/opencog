#include "DestinKernel.h"

// C/C++ headers
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdexcept>

// Cuda header
#include <cuda.h>
#include <curand.h>

const int AmountThreads = 128;


using namespace std;

__global__ void CalculateDistance( int States, int InputDimensionlity, float *InputData, float *CentroidVectorData, float *CentroidDist, float *CentroidStarvation);
__global__ void CalculateWinningCentroids( int States, float *CentroidDist, int *WinningCentroids );
__global__ void UpdateStarvation( int States, float StarvationCoefficient, int *WinningCentroids, float *CentroidStarvation );
__global__ void UpdateWinningCentroids( int States, int InputDimensionlity, float LearningRate, float *InputData, float *CentroidVectorData, int *WinningCentroids, float *CentroidDist );
__global__ void CalculatePOS( int States, float *CentroidDist, float *Output );
__global__ void UpdateBeliefs( const int states, float *dPOS, float * dNewBeliefs, float * dOldBeliefs, int * dCountingTables,	int * dParentsInputAdvice, int parentStates, int * dSumTables, bool isPSSATraining, int * dOutputAdvice);
__device__ void find_max(const int thread_start, const int threadcount, const int length,float * winner, int * winnerId );
__device__ void updateCountingTables(int mStates, int parentStates, int * dCountingTables, int advice, int previousWinningBelief, int newWinningBelief, int * dSumTables, int bid);
__global__ void initializeMemory( const int states, const int parentStates, float * dBeliefs, int * dOutputAdvice, int * dCountingTables, int * dSumTables );

DestinKernel::DestinKernel( void )
{
    mID=0;
	mRows=0;
	mCols=0;
	mStates=0;
	mParentStates=0;
	mInputDimensionlity=0;
    mLearningRate = 0;
    mSTARVATION_COEFFICIENT = 0;
	cuDeviceGetCount(&mDevices);
	dParentInputAdvice=NULL;
	mIsCentroidsTraining = true;
	mIsPSSATraining = true;
	cout << "Kernel created" << endl;
}

void DestinKernel::free(){
	//everything is set to null so in case free is called twice,
	//then it wont try to delete old dangling pointers.
    cudaFree( dCentroidsVectorData ); dCentroidsVectorData = NULL;
    cudaFree( dPOS ); dPOS = NULL;
    cudaFree( dCentroidsDistance ); dCentroidsDistance = NULL;
    cudaFree( dCentroidStarvation ); dCentroidStarvation  = NULL;
    cudaFree( dWinningCentroids ); dWinningCentroids = NULL;
    cudaFree( dBeliefs );dBeliefs = NULL;
    cudaFree( dOutputAdvice ); dOutputAdvice = NULL;
    cudaFree( dCountingTables );dCountingTables = NULL;
    cudaFree( dSumTables );dSumTables = NULL;

    delete [] mCentroidsDistance;mCentroidsDistance = NULL;
    delete [] mCentroidStarvation;mCentroidStarvation = NULL;
    delete [] mWinningCentroids;mWinningCentroids = NULL;
    delete [] mPOS;mPOS = NULL;
    delete [] mCentroidWinCounter;mCentroidWinCounter = NULL;
    delete [] mBeliefs;mBeliefs = NULL;

    cout << "Kernel destroyed" << endl;
}

DestinKernel::~DestinKernel( void )
{
	this->free();
}

#define CUDA_TEST_MALLOC( p, s )                                                                           \
	if( cudaMalloc( p , s ) != 0 ){                                                                        \
		 stringstream mess; mess << "could not cudaMaclloc at " << __FILE__ << ":" << __LINE__ << endl ;   \
         mess << "trying to allocate with size " << s << endl ;            		             		       \
	     throw runtime_error(mess.str());}                                                                 \

void DestinKernel::Create( int ID, int Rows, int Cols, int States, int ParentStates, int InputDimensionlity, float FixedLeaningRate, curandGenerator_t gen)
{
    mID = ID;
    mRows = Rows;
    mCols = Cols;
    mStates = States;
    mParentStates = ParentStates;
    mInputDimensionlity = InputDimensionlity;
    mLearningRate = FixedLeaningRate;
    if(ParentStates==0){
	throw logic_error("ParentStates must be at least one. Set = 1 if this is the top layer");
    }
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

    // Size of the layer with all vectors is rows times columns times centroids times the input (also observation) vector length.
    sizeOfLayerData = sizeOfNodeData*mInputDimensionlity;
    // Keep track of which centroid won
    mCentroidWinCounter = new int[sizeOfNodeData];
    for(int c=0;c<sizeOfNodeData;c++)
    {
        mCentroidWinCounter[c] = 0;
    }

    //TODO: put in error checking incase the cudaMallocs fail in case of not enough memory on device
    // Array full with all the winning centroids of each node
    mWinningCentroids = new int[sizeOfNodes];
    CUDA_TEST_MALLOC( (void**)&dWinningCentroids, sizeOfNodes*sizeof(int) );

    // Node data contains the distance to the observation of all centroids (It's is empty the first run)
    mCentroidsDistance = new float[sizeOfNodeData];
    CUDA_TEST_MALLOC( (void**)&dCentroidsDistance, sizeOfNodeData*sizeof(float) );

    // Starvation data for all centroids
    mCentroidStarvation = new float[sizeOfNodeData];
    CUDA_TEST_MALLOC( (void**)&dCentroidStarvation, sizeOfNodeData*sizeof(float) );
    for(int i=0;i<sizeOfNodeData;i++)
    {
        mCentroidStarvation[i]=1.0f;
    }
    // Copy the data from host to device
    cudaMemcpy(dCentroidStarvation, mCentroidStarvation, sizeOfNodeData*sizeof(float), cudaMemcpyHostToDevice);

    //TODO: make sure this POS is being fed to the correct place, and if it needs to go back to the host
    //POS - P(o|s') of update equation
    mPOS = new float[sizeOfNodeData];
    CUDA_TEST_MALLOC( (void**)&dPOS, sizeOfNodeData*sizeof(float) );

    CUDA_TEST_MALLOC( (void**)&dCentroidsVectorData, sizeOfLayerData*sizeof(float) );

    // This is to fill the dLayerData with all random numbers between 0.0 and 1.0
    curandGenerateUniform( gen, dCentroidsVectorData, sizeOfLayerData );

    //Node belief output, fed as input to parent nodes
    mBeliefs = new float[sizeOfNodeData];
    CUDA_TEST_MALLOC((void**)&dBeliefs, sizeOfNodeData * sizeof(float));
    
    //Node advice for fed to child nodes
    CUDA_TEST_MALLOC((void**)&dOutputAdvice, sizeOfNodes * sizeof(int));

    //Used in P(s'|s,a) calculations, counts when node transitions from s to s' when parent advice = a
    cout << "mRows " << mRows <<  "mCols " << mCols << "mParentStates " << mParentStates << " mStates " << mStates
    		<< " sizeof int " << sizeof(int) << endl ;
    CUDA_TEST_MALLOC((void**)&dCountingTables, mRows * mCols * mParentStates * mStates * mStates * sizeof(int));

    //Used in P(s'|s,a) (aka PSSA) calculations, holds the sum of the counting table columns
    CUDA_TEST_MALLOC((void**)&dSumTables, mRows * mCols * mParentStates * mStates * sizeof(int));

    dim3 grid(mCols, mRows); //grid of nodes (aka blocks)
    dim3 threads(AmountThreads);//threads per node

    //initialize memory to uniform distribution
    initializeMemory<<<grid, threads>>>(mStates, mParentStates, dBeliefs, dOutputAdvice, dCountingTables, dSumTables);
}

__global__ void initializeMemory( const int states, const int parentStates, float * dBeliefs, int * dOutputAdvice, int * dCountingTables, int * dSumTables ){
	int bid = blockIdx.x + blockIdx.y * gridDim.x;
	int tid = threadIdx.x;
	//dBeliefs[bid]
	int threads = blockDim.x ;

	const float uniform_c = 1.0 / (float)states;
	for(int t = tid ; t < states ; t+=threads ){
		dBeliefs[bid*states+t] = uniform_c;
	}

	if(tid==0){
		dOutputAdvice[bid] = 0;
	}

	//Each node has N=parentStates counting tables, each is size states x states
	for(int t = tid ; t < parentStates * states * states ; t+=threads){
		dCountingTables[bid * parentStates * states * states + t] = 1;
	}
	//holds the sums of the columns of the counting tables
	for(int t = tid ; t < parentStates * states ; t+=threads){
		dSumTables[bid*parentStates * states + t ] = states;
	}
}

void DestinKernel::DoDestin( float *Input, stringstream * xml )
{
    // Threads is the amount of thread inside each block
    dim3 threads( AmountThreads );
    // Grid is the amount of blocks inside a grid.
    dim3 grid( mCols, mRows );
    // Cause of the use of dynamic shared memory you have to tell the kernel how much shared memory space you need for each block.
    int sharedMem;
    // The launch of the kernels itself with centroids(states), dimension, input data and the Data of the layer itself
    // Calculating the distance of the centroids to an observation
    sharedMem = (mInputDimensionlity+mInputDimensionlity)*sizeof(float);
    CalculateDistance<<<grid, threads, sharedMem>>>( mStates, mInputDimensionlity, Input, dCentroidsVectorData, dCentroidsDistance, dCentroidStarvation );

    if(mIsCentroidsTraining){
		sharedMem = (mStates+mStates)*sizeof(float);
		// Kernel for finding the winning centroids
		CalculateWinningCentroids<<<grid, threads, sharedMem>>>( mStates, dCentroidsDistance, dWinningCentroids );
		// Kernel for starvation updates
		UpdateStarvation<<<grid, threads>>>( mStates, mSTARVATION_COEFFICIENT, dWinningCentroids, dCentroidStarvation );
		// Kernel for updating winning centroids
		sharedMem = mInputDimensionlity*sizeof(float);
		UpdateWinningCentroids<<<grid, threads, sharedMem>>>( mStates, mInputDimensionlity, mLearningRate, Input, dCentroidsVectorData, dWinningCentroids, dCentroidsDistance );
    }

    // Kernel for calculating P(o|s')
    sharedMem = (mStates+mStates)*sizeof(float);
    CalculatePOS<<<grid, threads, sharedMem>>>( mStates, dCentroidsDistance, dPOS );

    //TODO: rename dNewBeliefs and dOldBeliefs to just dBeliefs, get rid of one.
    int n = mStates > 16 ? 16 : mStates ;
    dim3 threads_plane( n, n ); 
    //total threads should be less than 512 per block, hardware limit so states needs to be less<=22
    //Chose 16 because seems like it would play better than 22... but not sure.
    //TODO: this should probably be a multiple of states instead to avoid wasting threads
    //TODO: might make sense to break up the UpdateBeliefs kernel because a lot is done with just a single row of threads so
    //lots of the threads are wasted, not sure if this would outweigh the overhead of a separate kernel launches
    sharedMem = (mStates * mStates + mStates) * sizeof(float);

    UpdateBeliefs<<<grid, threads_plane, sharedMem >>>(mStates, dPOS, dBeliefs, dBeliefs, dCountingTables, dParentInputAdvice, mParentStates, dSumTables, mIsPSSATraining, dOutputAdvice) ;
    

    this->WriteData(xml);
}

void DestinKernel::WriteData( stringstream * xmls )
{
    cudaMemcpy(mCentroidsDistance, dCentroidsDistance, sizeOfNodeData*sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(mCentroidStarvation, dCentroidStarvation, sizeOfNodeData*sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(mPOS, dPOS, sizeOfNodeData*sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(mWinningCentroids, dWinningCentroids, sizeOfNodes*sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(mBeliefs, dBeliefs, sizeOfNodeData * sizeof(float), cudaMemcpyDeviceToHost);

    if(xmls!=NULL){
    	stringstream & xml(*xmls);
        xml << "<layer id=\"" << mID << "\">" << endl;
        for(int r=0;r<mRows;r++)
        {
            for(int c=0;c<mCols;c++)
            {
                int winningCentroid = mWinningCentroids[r*mCols+c];
                // winning counter finds place on the host might not be the best place to put this still
                // cause we are already writing here some output why create a special loop for it.
                mCentroidWinCounter[(c+r*mCols)*mStates+winningCentroid] += 1;
                xml << "<node id=\"" << r*mCols+c << "\" centroidWin=\"" << mWinningCentroids[r*mCols+c] << "\">" << endl;
                for(int s=0;s<mStates;s++)
                {
                    xml << "<centroid id=\"" << s << "\" ";
                    xml << "lastDistance=\"" << mCentroidsDistance[(c+r*mCols)*mStates+s] << "\" ";
                    xml << "starvation=\"" << mCentroidStarvation[(c+r*mCols)*mStates+s] << "\" ";
                    xml << "POS=\"" << mPOS[(c+r*mCols)*mStates+s]  << "\" ";
                    xml << "winCount=\"" << mCentroidWinCounter[(c+r*mCols)*mStates+s]  << "\"";
                    xml << "/>" << endl;
                }
                xml << "</node>" << endl;
            }
        }
        xml << "</layer>" << endl;
    }

}
// ***********************
// DeSTIN inside CUDA Part
// ***********************
__global__ void CalculateDistance( int States, int InputDimensionlity, float *InputData, float *CentroidVectorData, float *CentroidDist, float *CentroidStarvation)
{
    // This is how to declare a shared memory inside CUDA.
    extern __shared__ float shared[];
    float* input = (float*)&shared;
    float* distance = (float*)&input[InputDimensionlity];

    // We use many threads they need to know where they have to do there work.
    // tid (Thread ID) is the amount of threads inside a block its a fixed amount it can be changed by changing: AmountThreads.
    // Keep in mind that CUDA threads should be in steps of 32 (each warp takes 4 clock cycles where each cycle calculate 8 threads)
    int tid = threadIdx.x;
    // bid (Block ID) this keeps track in which  node we are working you can ask the grid the size of the blocks used in x or y and on a Fermi or higher even z
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
    //TODO: should be able to parallelize over states
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
            temp = input[tid] - CentroidVectorData[InputDimensionlity*States*bid+centroid*InputDimensionlity+tid] ;
            distance[tid] = temp * temp;
            // A trick for when the dimension is bigger then the amount of threads
            tid += blockDim.x;
        }
        // all threads have to wait here so we know all distance have been calculated
        __syncthreads();

        // Cause DeSTIN don't work with numbers that are 2^? we have to check for odd numbers
        int dOld = InputDimensionlity;
        // bite wise divide by 2 (should be faster the /2)
        int d = InputDimensionlity >> 1;
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

                // special case in case of odd number (As long as this doesn't happen too often it won't effect speed)
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
        if(tid == 0)
        {
            // square root on sum of the (input - centroid)*(input - centroid)
            // (Remember that you should copy the data from the device to the host and store it then)
            CentroidDist[centroid+bid*States] = (sqrt(distance[tid]))*CentroidStarvation[centroid+bid*States];
        }
        // go to next centroid inside the node (bid is taking care of the other node)
        centroid++;
    }
}

	// To reduce the amount of work that one kernel is doing i have decided that splitting the work over more kernels should speed up the whole procces
   //TODO: make sure this can work with __shared__ as is
__global__ void CalculateWinningCentroids( int States, float *CentroidDist, int *WinningCentroids )
{
    extern __shared__ float shared[];
    float* winner = (float*)&shared;
    float* winnerId = (float*)&winner[States];
    int tid = threadIdx.x;
    int bid = blockIdx.x + blockIdx.y * gridDim.x;

    while(tid < States)
    {
        winnerId[tid] = tid;
        winner[tid] = CentroidDist[tid+bid*States];
        tid += blockDim.x;
    }
    __syncthreads();

    int dOld = States;
    int d = States >> 1;
    while (d != 0)
    {
        tid = threadIdx.x;
        dOld = dOld - d*2;
        while(tid < d)
        {
            if(winner[tid] > winner[tid + d])
            {
                // Move winning centroid to the beginning
                winner[tid] = winner[tid + d];
                winnerId[tid] = winnerId[tid + d];
            }

            if (dOld == 1 && tid == d-1)
            {
                // special case of odd numbers
                if(winner[tid] > winner[tid + d + 1])
                {
                    winner[tid] = winner[tid + d + 1];
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

// This is the updating starvation fast and quick to update all the nodes and reset the winning centroid
// According to DeSTIN paper: The winning centroid starvation gets reset while the others starve more
// Aldo this is the simple version of it it might be changed in the further cause this make the network also forget what it learn
// when it is looking at something else for a very long time (Short and Long term memory)
__global__ void UpdateStarvation( int States, float StarvationCoefficient, int *WinningCentroids, float *CentroidStarvation )
{
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

// Move the winning centroids closer to the observation
__global__ void UpdateWinningCentroids( int States, int InputDimensionlity, float LearningRate, float *InputData, float *CentroidVectorData, int *WinningCentroids, float *CentroidDist )
{
    extern __shared__ float newDistance[];
    int tid = threadIdx.x;
    int bid = blockIdx.x + blockIdx.y * gridDim.x;

    int centroid = WinningCentroids[bid];
    float temp;
    float inputD;
    int pos;

    //this while block calculates the distance between the input vectors
    //and the centroid vectors
    while(tid < InputDimensionlity)
    {
        pos = InputDimensionlity*States*bid+centroid*InputDimensionlity+tid;
        temp = CentroidVectorData[pos];
        inputD = InputData[tid + bid * InputDimensionlity];
        temp = inputD - (temp * LearningRate);
        CentroidVectorData[pos] = temp;
        temp = (inputD - temp) * (inputD - temp);
        newDistance[tid] = temp;

        tid += blockDim.x;
        pos += blockDim.x;
    }
    __syncthreads();

    int dOld = InputDimensionlity;
    int d = InputDimensionlity >> 1;
    while (d != 0)
    {
        tid = threadIdx.x;
        dOld = dOld - d*2;
        while(tid < d)
        {
            newDistance[tid] += newDistance[tid + d];
            if (dOld == 1 && tid == d-1)
            {
                // special case of odd numbers
                newDistance[tid] += newDistance[tid + d + 1];
            }
            tid += blockDim.x;
        }
        // Sync moment before starting with next iteration of reduction.
        __syncthreads();

        dOld = d;
        d >>= 1;
    }

    tid = threadIdx.x;
    if(tid == 0)
    {
        CentroidDist[centroid+bid*States] = sqrt(newDistance[0]);
    }
}

__global__ void CalculatePOS( int States, float *CentroidDist, float *POSOutput )
{
    extern __shared__ float shared[];
    float* distance = (float*)&shared;
    float* tPOS = (float*)&distance[States];
    int tid = threadIdx.x;
    int bid = blockIdx.x + blockIdx.y * gridDim.x;

    while(tid < States)
    {
        distance[tid] = CentroidDist[bid*States+tid];
        tPOS[tid] = (float)(1.0/(1e-9+(double)distance[tid]));
        tid += blockDim.x;
    }
    __syncthreads();

    int dOld = States;
    int d = States >> 1;
    while (d != 0)
    {
        tid = threadIdx.x;
        dOld = dOld - d*2;
        while(tid < d)
        {
            tPOS[tid] += tPOS[tid + d];
            if (dOld == 1 && tid == d-1)
            {
                // special case of odd numbers
                tPOS[tid] += tPOS[tid + d + 1];
            }
            tid += blockDim.x;
        }
        // Sync moment before starting with next iteration of reduction.
        __syncthreads();

        dOld = d;
        d >>= 1;
    }

    tid = threadIdx.x;
    while(tid < States)
    {
        // This is the POS for all centroids (It looks like this is the input for the next layer also)
        // The output is missing the advice of higher layer
    	POSOutput[tid+bid*States] = (float)(1.0/(1e-9+(double)distance[tid]))/tPOS[0];
        tid += blockDim.x;
    }
}

/**
 * UpdateBeliefs - Performs the P(s'|s,a)*b(s) calculations of the DeSTIN belief update rule.
 *
 * PSSA means P(s'|s,a)*b(s) where a = advice, meaning
 * probability of transitioning to state s' given the current state s and the parents node's advice (or state) a.
 * Each node has a separate counting table for each possible parent advice state
 * of size N x N, where N is the number of centroids (states) of the child node.
 * The number of counting tables per node equal to the number of parent states.
 * Each time a node transitions from s to s' given advice a, the counting table for advice a
 * has the value of the element at row s and column s' incremented by 1. Then, to get the
 * probability, that value is divided by the corresponding value in the SumsTables.
 * There is one sum table per counting table, which has one element per column of the
 * matching counting table which sums up the elements of the column
 *
 * states - number of node centroids
 * dPOS - P(o|s') calculated from CalculatePOS kernel
 * dNewBeliefs - b'(s') - updated beliefs. The node output, fed to parent nodes as input
 * dOldBeliefs - b(s) - beliefs how they were before calling this kernel, currently dOldBeliefs points to same memory location as dNewBeliefs
 * dCountingTables - keeps track of the P(s'|s,a) table along with the dSumTables
 * dParentsInputAdvice - input advice from the parent node. The 'a' of P(s'|s,a). NULL if this is the top layer, no parent layer.
 * parentStates - number of centroids of the parent node. Expects it to be set to 1 if this is the top layer.
 * dSumTables - vector of the sum of the columns of the dCountingTables
 * isPSSATraining - if true, the PSSA counting tables used to calculate P(s'|s,a) are updated
 * dOutputAdvice - this node's advice to be fed to its children nodes
 */
//TODO: make a check to see if it has enough shared memory
__global__ void UpdateBeliefs( const int states, float *dPOS, float * dNewBeliefs, float * dOldBeliefs, int * dCountingTables,
		int * dParentsInputAdvice, int parentStates, int * dSumTables, bool isPSSATraining, int * dOutputAdvice){

	int bid = blockIdx.x + blockIdx.y * gridDim.x; //corresponds to the node

	//TODO: enforce square layers or update child to parent mapping code to handle non square layers
	//Points the 4 children nodes to the right parent for advice
	//Be careful of integer division if trying to simplify this.
	//TODO: simplify this
	int parent_node_id = blockIdx.x / 2 + (blockIdx.y /2 ) * (gridDim.x / 2) ;
	//TODO: could make one thread pull from global memory into shared memory, instead of all threads pulling
	const int advice = dParentsInputAdvice==NULL ? 0 : dParentsInputAdvice[parent_node_id];

	const int s2 = states * states;

	// Variable cts (counting table start) is the first element (at 0,0) of the correct PSSA counting table
	// based on the node and advice state.
	const int cts = bid * parentStates * s2 + advice * s2;


	extern __shared__ float cache[]; // the cache saves each P(s'|s,c)*b(s) for all s' and s for the given advice c. The size  is states x states ( plus another states tacked on the end later, see kernel launch params)

	//variable sp is read as "s prime" as in b'(s') which is the left side of the belief update equation.
	for(int sp = threadIdx.y; sp < states ; sp += blockDim.y) {
		int ctr = cts + sp * states; // ctr (counting table row) is the first element of the sp'th row of the a'th=advice counting table
		//s = current state
		for(int s = threadIdx.x ;  s < states ; s += blockDim.x ) {
			int i = ctr + s;//the s' row by s column element of the counting table
			//TODO: i might be performing this multiplication in the wrong order
			float prob = (float)dCountingTables[i] / (float) dSumTables[bid * parentStates * states + advice * states + s];
			//TODO: should probably save the dOldBeliefs vector to a shared memory variable first
			// to prevent having to pull it from global memory N=states times
			cache[ sp * states + s] = dOldBeliefs[bid * states + s] * prob; // this is the P(s'|s,c)*b(s) calculation.
		}
	}
	__syncthreads();



	//this part performs a reduction on the sums of the P(s'|s,c)*b(s) rows
	//of the cache table, storing the sums in the first column of the table.
	int dOld = states;
	for (int d = states >> 1;  d != 0; d >>= 1) { 				
		dOld -= d*2;	
		for(int sp = threadIdx.y; sp < states ; sp += blockDim.y){
			for(int s = threadIdx.x; s < d ; s += blockDim.x){
				int i = sp * states + s;
				cache[i] +=  cache[i + d];
				//trick for if cache has odd length
				if(dOld == 1 && s == d - 1){
					cache[i] += cache[i + d + 1];
				}
			}
		}
		__syncthreads(); 
		dOld = d;
	}

	float * pssc_b_vector = (float *)&cache[s2]; //length is states, start this vector right after the end of cache array

	//multiply the two parts of the belief update equation numerator together, Pr(o|s') by Sum[ Pr(s'|s,c)*b(S) ]
	//The cache[sp * states] is the Pr(s'|s,c)*b(S)  vector

	//we launched with a 2d block of threads now only dealing with 1d arrays, so convert this back to 1d so we waste fewer threads
	int sp_start  = threadIdx.y * blockDim.x + threadIdx.x; 
	int n_threads = blockDim.x * blockDim.y;

	//transform it from a column into a row
	for(int sp = sp_start ; sp < states ; sp += n_threads ){
		pssc_b_vector[sp] = cache[sp * states] *= dPOS[bid * states + sp];
	}
	__syncthreads(); //TODO: might not need this here


	//find the sum of the pssc_b_vector so it can be normalized   
	dOld = states;
	int s_start = sp_start;   
	for (int d = states >> 1; d!=0 ; dOld = d, d>>=1 ){
		dOld -= d*2;
		for(int s = s_start; s < d; s +=  n_threads ){
			pssc_b_vector[s] += pssc_b_vector[s + d ];
			if(dOld == 1 && s ==  d - 1){
				pssc_b_vector[s] += pssc_b_vector[s + d + 1];
			}
		}
		__syncthreads();	
	}

	float sum = pssc_b_vector[0];


	//pssc_b_vector will now contain b'(s') after normalizing the Pr(s'|s,c)*b(S) vector
	for(int sp = sp_start; sp < states ; sp += n_threads ){
		dNewBeliefs[ bid * states + sp] =  pssc_b_vector[sp] = cache[sp * states] /= sum;
	}

	//Update the p(s'|s,a) counting tables, incrementing the s column and s' row of the a'th counting table by one,
	//and incrementing the s column of the a'th sum tables by one
	int * max_index = (int *)cache; //max_index size = #states. Overwrite first row of cache shared memory to save winning index.
	//find max belief, store corresponding index in max_index[0]
	find_max(sp_start, n_threads, states, pssc_b_vector, max_index);
	//set max belief state as advice for child nodes
	if(sp_start == 0){//only one thread does this to save memory bandwidth
		int old_winning_belief = dOutputAdvice[bid];
		//new winning belief
		dOutputAdvice[bid] = max_index[0];
		if(isPSSATraining){
			updateCountingTables(states, parentStates, dCountingTables, advice, old_winning_belief, max_index[0], dSumTables, bid);
		}
	}

}

__device__ void updateCountingTables(int mStates, int parentStates, int * dCountingTables,
		int advice, int previousWinningBelief, int newWinningBelief, int * dSumTables, int bid){
	int s2 = mStates * mStates;
	//make sure Im consistent with old states across the top and new states down the side for the table.
	int i = bid * parentStates * s2 //node
		+ advice * s2 //advice table for node
		+ newWinningBelief * mStates //row of table
		+ previousWinningBelief;	//col of table

	dCountingTables[i]++;

	//dSumTables, collection of 1 dimensional vectors. Each node has the same number of them as its counting tables or one for each parent state.
	//One sum vector has length equal to the number of the node's centroids or states. Each element is the sum of the corresponding column
	//of the dCountingTable
	i = bid * parentStates * mStates // node index
		+ advice * mStates //advice index
		+ previousWinningBelief; //element of sum vector
	dSumTables[i]++;

}
/**
 * find_max - finds the maximum value of winner and its corresponding index. The 
 * maximum value is stored at winner[0] and the corresponding index of the maximum
 * value is stored at winnerId[0]
 * 
 * thread_start = tid
 * threadcount = how many threads
 * length = size of vector to find the maximum of
 * winner = input vector to find maximum of, winner[0] will contain the maximum value afterwords
 * winnerId = empty buffer to be used for scratch, winnerId[0] will have the max id afterwards
 */
__device__ void find_max(const int thread_start, const int threadcount, const int length,float * winner, int * winnerId ){

	int tid;
	for(tid = thread_start; tid < length ; tid += threadcount ){
		winnerId[tid] = tid;
	}
	__syncthreads();

	for(int dOld = length, d = length >> 1; d != 0 ; dOld = d, d >>= 1 ){
		for(tid = thread_start , dOld -= d*2 ; tid < d ; tid += threadcount){
			int tidd= tid + d;	
			if(winner[tid] < winner[tidd]){
				// Move large index to the beginning
				winner[tid] = winner[tidd];
				winnerId[tid] = winnerId[tidd];
			}
			if (dOld == 1 && tid == d-1){
				// special case of odd numbers
				if(winner[tid] < winner[tidd + 1]){
					winner[tid] = winner[tidd + 1];
					winnerId[tid] = winnerId[tidd + 1];
				}
			}
		}
		// Sync moment before starting with next iteration of reduction.
		__syncthreads();
	}
}

