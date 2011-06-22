#ifndef DESTIN_KERNEL_H
#define DESTIN_KERNEL_H
#include <cuda.h>
#include <curand.h>

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
     */
    int mID;
    int mRows;
	int mCols;
	int mStates;
	int mInputDimensionlity;
	int mDevices;
	float mSTARVATION_COEFFICIENT;

	int sizeOfLayerData;
	int sizeOfNodeData;
	int sizeOfNodes;

	float *mCentroidVectorData;
	float *dCentroidVectorData;
	float *mCentroidData;
	float *dCentroidData;
	float *mCentroidStarvation;
	float *dCentroidStarvation;
	int *mWinningCentroids;
	int *dWinningCentroids;
        
public:
	DestinKernel( void );

	~DestinKernel( void );

	/*
	 * Create a DeSTIN kernel here the layer and node and clustering is put all together.
	 *
	 */
	void Create( int ID, int Rows, int Cols, int States, int InputDimensionlity, curandGenerator_t gen);

	/*
	 * Do DeSTIN is the launcher of the GPU kernel.
	 * @param *Input It's input can be a dLayerData or a image(for lowest layer)
	 */
	void DoDestin( float *Input );

	int GetID(){ return mID; }

	int GetNumberOfRows(){ return mRows; }

	int GetNumberOfCols(){ return mCols; }

	int GetNumberOfStates(){ return mStates; }

	int GetNumberOfDevices(){ return mDevices; }

	int GetNumberOfInputDimensionlity(){ return mInputDimensionlity; }
};

#endif
