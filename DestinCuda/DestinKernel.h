#ifndef DESTIN_KERNEL_H
#define DESTIN_KERNEL_H

class DestinKernel
{
private:
    /*
     * mRows and mCols are the layer information.
     * mStates and mInputDemensionlity come from nodes.
     * mDevices contain the number of CUDA capable devices.
     * dLayerData is the pointer for GPU memory (It's empty as long as you don't use cudaMemcpy(to, from, size, direction))
     */
    int mRows;
	int mCols;
	int mStates;
	int mInputDimensionlity;
	int mDevices;
	float *dLayerData;
        
public:
	DestinKernel( void );

	~DestinKernel( void );

	void Create( int Rows, int Cols,	int States, int InputDimensionlity );

	void DoDestin( float *image );

	int GetNumberOfRows(){ return mRows; }

	int GetNumberOfCols(){ return mCols; }

	int GetNumberOfStates(){ return mStates; }

	int GetNumberOfDevices(){ return mDevices; }

	int GetNumberOfInputDimensionlity(){ return mInputDimensionlity; }
};

#endif
