#ifndef DESTIN_KERNEL_H
#define DESTIN_KERNEL_H

class DestinKernel
{
private:
    int mRows;
	int mCols;
	int mStates;
	int mInputDimensionlity;
	int mDevices;
	float * dLayerData;
        
public:
	DestinKernel(void);

	~DestinKernel(void);

	void Create(int Rows, int Cols,	int States, int InputDimensionlity);

	void DoDestin(float *image);

	int GetNumberOfRows(){ return mRows;}

	int GetNumberOfCols(){ return mCols;}

	int GetNumberOfStates(){ return mStates;}

	int GetNumberOfDevices(){ return mDevices;}
};

#endif
