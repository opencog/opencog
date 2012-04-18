#ifndef DESTIN_KERNEL_H
#define DESTIN_KERNEL_H
// C/C++ headers
#include <sstream>
// Cuda header
#include <cuda.h>
#include <curand.h>

using namespace std;

/*
 * DeSTIN Kernel
 * This now the before layer,nodes,centroids,clustering system of DeSTIN
 * When started to create this one i did not find it necessarily to create all the classes for it.
 * Although it should be possible to create the old structure of files back and make sure you pass all the pointers to the kernel.
 * This would have been done if the research did not take so long to understand DeSTIN original version.
 * You need to open the DestinKernel.cu to see the actual Kernels
 * This because they have to be more like C and creating special CUDA headers is most of the time avoided (CUDA forums)
 */
class DestinKernel
{
private:
    /*
     * mID is the ID of the layer
     * mRows and mCols are the layer size information.
     * mStates(centroids) and mInputDemensionlity are node information.
     * mDevices contain the number of CUDA capable devices.
     * mLayerData is the pointer for CPU memory so we can copy from CPU to GPU and back
     * dLayerData is the pointer for GPU memory (It's empty as long as you don't use cudaMemcpy(to, from, size, direction))
     * mSTARVATION_COEFFICIENT this came from original DeSTIN and was been used for updating starvation
     *
     * size of Layer Data/Node Data/Nodes this is many been used for data copying between Host and Device
     * Pointers: where pointer starting with a m is on Host and d is on the device
     */
    int mID;
    int mRows;
	int mCols;
	int mStates;
	int mParentStates;
	int mInputDimensionlity;
	int mDevices;
	float mLearningRate;
	float mSTARVATION_COEFFICIENT;

	int sizeOfLayerData;
	int sizeOfNodeData;
	int sizeOfNodes;

	float *dCentroidsVectorData;
	float *mCentroidsDistance;
	float *dCentroidsDistance;
	float *mCentroidStarvation;
	float *dCentroidStarvation;
	float *mPOS;
	float *dPOS;

	int *mWinningCentroids;
	int *dWinningCentroids;
	int *mCentroidWinCounter;

	//TODO: properly document how the memory variables are arranged.

	int *dCountingTables;
	int *dSumTables;
	float *dBeliefs;
	float *mBeliefs;
        
	//TODO: make sure all memory vectors are initialized and cleared properly
	int *dOutputAdvice; //advice to be used by child layer

	int *dParentInputAdvice;//points to advice vector in parent layer, child layer is not responsible for clearing it

	void WriteData( stringstream * xml );


	bool mIsCentroidsTraining;
	bool mIsPSSATraining;

public:
	DestinKernel( void );

	~DestinKernel( void );

	//need this method so graphics memory can be free
	//from other language wrappers that use garbage collection.
	//the user can specifically release graphics card memory instead
	//of hoping the gc will eventually call the destructor to free graphics memory
	void free();

	/*
	 * Create a DeSTIN kernel here the layer and node and clustering is put all together.
	 */
	void Create( int ID, int Rows, int Cols, int States, int ParentStates, int InputDimensionlity, float FixedLeaningRate, curandGenerator_t gen );



	/*
	 * Do DeSTIN is the launcher of the GPU kernel.
	 * @param *Input It's input can be a dNodeOutput or a image(for lowest layer)
	 * Make sure it is the device pointer and not the host pointer (kernel will crash/not run with it)
	 *
	 * xml - string buffer to out
	 */
	void DoDestin( float *Input, stringstream * xml);

	// Mostly getters i name should be clear enough
	int GetID(){ return mID; }

	int GetNumberOfRows(){ return mRows; }

	int GetNumberOfCols(){ return mCols; }

	int GetNumberOfStates(){ return mStates; }

	int GetNumberOfDevices(){ return mDevices; }

	int GetNumberOfInputDimensionlity(){ return mInputDimensionlity; }

	float *GetDevicePointerPOS(){ return dPOS; }

	float *GetDevicePointerBeliefs(){ return dBeliefs; }
        
	int * GetOutputAdvice(){ return dOutputAdvice;  }

	void SetInputAdvice(int * dParentInputAdvice){
		this->dParentInputAdvice = dParentInputAdvice;
	}

	/**
	 * setIsPOSTraining
	 *
	 * when set to true, the centroids used in online clustering
	 * for calculation of the P(o|s') of the belief
	 * update equation are continually updated with
	 * each new input. Default is true.
	 */
	void setIsPOSTraining(bool is_training){
		mIsCentroidsTraining = is_training;
	}

	/**
	 * setIsPSSATraining
	 *
	 * When set to true, the PSSA tables used in the
	 * P(s'|s,a) portion of the belief update equation
	 * are continually updated with each new input.
	 * Default is true.
	 */
	void setIsPSSATraining(bool is_training){
		mIsPSSATraining = is_training;
	}

	/**
	 * Returns a host pointer that points to the beginning of the node's belief state
	 * vector b(s) of the DeSTIN belief update equation.
	 * 
	 * Undefined behavior if nodeRow and nodeCol are out of bounds.
	 * 
	 * @param layer - which layer the node belongs to
	 * @param nodeRow - row the node is in of the given layer
	 * @param nodeCol - col the node is in of the given layer
	 * @return - a pointer that points to the beginning of the chosen
	 * node's belief state vector. The length of the vector can be found
	 * using the GetNumberOfStates() method.
	 * 
	 */
	float * GetNodeBeliefVector(int nodeRow, int nodeCol){
		return &mBeliefs[nodeRow*mCols + nodeCol];
	};

};
#endif
