#ifndef DESTIN_KERNEL_H
#define DESTIN_KERNEL_H

class DestinKernel
{
private:
    /*
     * mID is the ID of the layer
     * mRows and mCols are the layer size information.
     * mStates(centroids) and mInputDemensionlity are node information.
     * mDevices contain the number of CUDA capable devices.
     * mLast tells if it is the last layer
     * mLayerData is the pointer for CPU memory so we can copy from CPU to GPU and back
     * dLayerData is the pointer for GPU memory (It's empty as long as you don't use cudaMemcpy(to, from, size, direction))
     */
    int mID;
    int mRows;
	int mCols;
	int mStates;
	int mInputDimensionlity;
	int mDevices;
	bool mLast;
	float *mLayerData;
	float *dLayerData;
        
public:
	DestinKernel( void );

	~DestinKernel( void );

	void Create( int ID, int Rows, int Cols, int States, int InputDimensionlity, bool Last );

	void DoDestin( float *image );

	int GetID(){ return mID; }

	int GetNumberOfRows(){ return mRows; }

	int GetNumberOfCols(){ return mCols; }

	int GetNumberOfStates(){ return mStates; }

	int GetNumberOfDevices(){ return mDevices; }

	int GetNumberOfInputDimensionlity(){ return mInputDimensionlity; }

	bool GetInfoLast(){ return mLast; }
};

#endif
