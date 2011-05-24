#ifndef DESTIN_KERNEL_H
#define DESTIN_KERNEL_H

class DestinKernel
{
private:
    int mRows;
	int mCols;
	int mStates;
	int mDevices;
        
public:
	DestinKernel(void);

	~DestinKernel(void);

	void Create(int Rows, int Cols,	int States);

	int GetNumberOfRows(){ return mRows;}

	int GetNumberOfCols(){ return mCols;}
};

#endif
