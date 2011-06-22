#include "DestinData.h"

#include <fstream>
#include <map>
#include <math.h>
#include <iostream>

using namespace std;

DestinData::DestinData(void)
{
	mLastImageIndex=-1;
	mRows=0;
	mCols=0;
	// TODO: This is also not dynamic works for numbers of 32x32 images
    for(int r=0;r<40;r++)
    {
        for(int c=0;c<40;c++)
        {
            mImageWithOffset[r][c]=0;
        }
    }
	cout << "Destin data created" << endl;
}

DestinData::~DestinData(void)
{
	cudaFree(dImage);
	delete [] mImage;
	cout << "Destin data deleted" << endl;
}

void DestinData::LoadFile(const char* sFileName)
{
	mImagePointer.clear();
	mLabels.clear();
	mUniqueLabels.clear();
	mMapLabelToIndexVector.clear();

	std::ifstream stmIn;
	stmIn.open(sFileName,ios::in | ios::binary );

	int iSignals;
	stmIn.read( (char*)&iSignals,sizeof(iSignals));
	stmIn.read( (char*)&mRows, sizeof(mRows) );
	stmIn.read( (char*)&mCols, sizeof(mCols) );
	// Create the array for pinned memory for CUDA
    int size = mRows*mCols;
    // Host side memory
    mImage = new float[size];
    // Device side memory
    cudaMalloc( (void**)&dImage, size*sizeof(float) );

	int iLabel;
	unsigned char* cImageData;
	float** fImageDataByRow;
	float *fRow;

	map<int, vector<int> >::iterator it;

	for(int i=0; i<iSignals; i++)
	{
		stmIn.read( (char*)&iLabel, sizeof(iLabel) );
		mLabels.push_back(iLabel);
		it = mMapLabelToIndexVector.find(iLabel);
		if ( it==mMapLabelToIndexVector.end() ) //didn't find it
		{
			vector<int> vInt;
			vInt.push_back((int)(mLabels.size())-1);
			mMapLabelToIndexVector[iLabel]=vInt;
			mUniqueLabels.push_back(iLabel);
		}
		else
		{
			it->second.push_back((int)(mLabels.size())-1); //add the new vectors index...
		}
		cImageData = new unsigned char[mRows*mCols];
		stmIn.read( (char*)cImageData, mRows*mCols );  //The data is stored row 1, column 1-end, row 2, column 1-end, etc
		// so we have to transpose it here...
		fImageDataByRow = new float*[mRows];
		unsigned char* p = cImageData;
		for(int r=0;r<mRows;r++)
		{
			fRow = new float[mCols];
			for(int c=0;c<mCols;c++)
			{
				fRow[c]=((float)*p)/255.0;
				p++;
			}
			fImageDataByRow[r]=fRow;
		}
		delete cImageData;
		mImagePointer.push_back(fImageDataByRow);
	}
	stmIn.close();
	//cudaMalloc();
	cout << "Finished reading file." << endl;
}

void DestinData::SetShiftedDeviceImage(int ImageIndex, int RowShift, int ColShift, int DemRow, int DemCol)
{
    // TODO: Might want to set C and R more dynamic. in case of different data set?
    int C = 4;
    int R = 4;
    // We don't have to load the image if it is the same one as before
    if ( ImageIndex!=mLastImageIndex )
    {
        // Load the image into the buffer with the "R,C" offset.
        float** fImage = mImagePointer[ImageIndex];
        for(int r=0;r<mRows;r++)
        {
            for(int c=0;c<mCols;c++)
            {
                mImageWithOffset[r+R][c+C]=fImage[r][c];
            }
        }
    }
    // Now load the data using the offset provided.
    // Convert a 2D array back to a 1D array
    int i = 0;
    for(int row=0;row<mRows;row+=DemRow)
    {
        for(int col=0;col<mCols;col+=DemCol)
        {
            // To optimize the memory use inside CUDA put the DemRow*DemCol as one block.
            // This makes the kernel do the same for all layers also.
            for(int r=0;r<DemRow;r++)
            {
                for(int c=0;c<DemCol;c++)
                {
                    mImage[i]=mImageWithOffset[r+row+RowShift][c+col+ColShift];
                    i++;
                }
            }
        }
    }
    // Copy data from host to device
    cudaMemcpy( dImage, mImage, mRows*mCols*sizeof(float), cudaMemcpyHostToDevice );
    mLastImageIndex=ImageIndex;
}

void DestinData::GetLabelList(vector<int>& Labels)
{
    Labels.clear();
    for(int i=0;i<(int)(mLabels.size());i++)
    {
        Labels.push_back(mLabels[i]);
    }
}

void DestinData::GetUniqueLabels(vector<int>& vUniqueLabels)
{
    vUniqueLabels.clear();
	vector<int>::iterator it = this->mUniqueLabels.begin();
	while ( it != mUniqueLabels.end() )
	{
	    vUniqueLabels.push_back(*it++);
	}

}

void DestinData::GetIndicesForThisLabel(int iLabel, vector<int>& IndicesForThisLabel )
{
	IndicesForThisLabel.clear();

	map<int, vector<int> >::iterator it;

	it = mMapLabelToIndexVector.find(iLabel);
	if ( it!=mMapLabelToIndexVector.end() ) //find it
	{
		vector<int>::iterator vit;
		vit = it->second.begin();
		while ( vit != it->second.end() )
		{
			IndicesForThisLabel.push_back( *vit++ );
		}
	}
}

void DestinData::DoSpecial4x4FFT(float** &fSubImage, float* fUniqueVector)
{
	// I don't generally recommend doing FFTs like this, but since we only want 4x4 I didn't want to 
	// go to the trouble of adding an external library.  Plus you can do 4x4 with only adds & subtracts...
	int r,c;
	float fV;
	//Initialize the imaginary parts that we won't be 'hitting'...
	mIP[0][0]=0;mIP[2][0]=0;mIP[0][2]=0;mIP[2][2]=0;
	////////////////Row 0 Col 0///////
	fV = fSubImage[0][0];
	mRP[0][0]=fV; mRP[0][0]=fV; mRP[1][0]=fV; mRP[1][0]=fV; mRP[2][0]=fV; mRP[2][0]=fV; mRP[3][0]=fV; mRP[3][0]=fV; 
	mRP[0][1]=fV; mRP[0][1]=fV; mRP[1][1]=fV; mRP[1][1]=fV; mRP[2][1]=fV; mRP[2][1]=fV; mRP[3][1]=fV; mRP[3][1]=fV; 
	mRP[0][2]=fV; mRP[0][2]=fV; mRP[1][2]=fV; mRP[1][2]=fV; mRP[2][2]=fV; mRP[2][2]=fV; mRP[3][2]=fV; mRP[3][2]=fV; 
	mRP[0][3]=fV; mRP[0][3]=fV; mRP[1][3]=fV; mRP[1][3]=fV; mRP[2][3]=fV; mRP[2][3]=fV; mRP[3][3]=fV; mRP[3][3]=fV; 
	////////////////Row 0 Col 1///////
	fV = fSubImage[0][1];
	mRP[0][0]=mRP[0][0]+fV; mRP[1][0]=mRP[1][0]+fV; mRP[2][0]=mRP[2][0]+fV; mRP[3][0]=mRP[3][0]+fV; 
	mRP[0][2]=mRP[0][2]-fV; mRP[1][2]=mRP[1][2]-fV; mRP[2][2]=mRP[2][2]-fV; mRP[3][2]=mRP[3][2]-fV; 
	mIP[0][1]=-fV; mIP[0][1]=-fV; mIP[1][1]=-fV; mIP[1][1]=-fV; mIP[2][1]=-fV; mIP[2][1]=-fV; mIP[3][1]=-fV; mIP[3][1]=-fV; 
	mIP[0][3]=fV; mIP[0][3]=fV; mIP[1][3]=fV; mIP[1][3]=fV; mIP[2][3]=fV; mIP[2][3]=fV; mIP[3][3]=fV; mIP[3][3]=fV; 
	////////////////Row 0 Col 2///////
	fV = fSubImage[0][2];
	mRP[0][0]=mRP[0][0]+fV; mRP[1][0]=mRP[1][0]+fV; mRP[2][0]=mRP[2][0]+fV; mRP[3][0]=mRP[3][0]+fV; 
	mRP[0][1]=mRP[0][1]-fV; mRP[1][1]=mRP[1][1]-fV; mRP[2][1]=mRP[2][1]-fV; mRP[3][1]=mRP[3][1]-fV; 
	mRP[0][2]=mRP[0][2]+fV; mRP[1][2]=mRP[1][2]+fV; mRP[2][2]=mRP[2][2]+fV; mRP[3][2]=mRP[3][2]+fV; 
	mRP[0][3]=mRP[0][3]-fV; mRP[1][3]=mRP[1][3]-fV; mRP[2][3]=mRP[2][3]-fV; mRP[3][3]=mRP[3][3]-fV; 
	////////////////Row 0 Col 3///////
	fV = fSubImage[0][3];
	mRP[0][0]=mRP[0][0]+fV; mRP[1][0]=mRP[1][0]+fV; mRP[2][0]=mRP[2][0]+fV; mRP[3][0]=mRP[3][0]+fV; 
	mRP[0][2]=mRP[0][2]-fV; mRP[1][2]=mRP[1][2]-fV; mRP[2][2]=mRP[2][2]-fV; mRP[3][2]=mRP[3][2]-fV; 
	mIP[0][1]=mIP[0][1]+fV; mIP[1][1]=mIP[1][1]+fV; mIP[2][1]=mIP[2][1]+fV; mIP[3][1]=mIP[3][1]+fV; 
	mIP[0][3]=mIP[0][3]-fV; mIP[1][3]=mIP[1][3]-fV; mIP[2][3]=mIP[2][3]-fV; mIP[3][3]=mIP[3][3]-fV; 
	////////////////Row 1 Col 0///////
	fV = fSubImage[1][0];
	mRP[0][0]=mRP[0][0]+fV; mRP[2][0]=mRP[2][0]-fV; mRP[0][1]=mRP[0][1]+fV; mRP[2][1]=mRP[2][1]-fV; 
	mRP[0][2]=mRP[0][2]+fV; mRP[2][2]=mRP[2][2]-fV; mRP[0][3]=mRP[0][3]+fV; mRP[2][3]=mRP[2][3]-fV; 
	mIP[1][0]=-fV; mIP[1][0]=-fV; mIP[3][0]=fV; mIP[3][0]=fV; mIP[1][1]=mIP[1][1]-fV; mIP[3][1]=mIP[3][1]+fV; 
	mIP[1][2]=-fV; mIP[1][2]=-fV; mIP[3][2]=fV; mIP[3][2]=fV; mIP[1][3]=mIP[1][3]-fV; mIP[3][3]=mIP[3][3]+fV; 
	////////////////Row 1 Col 1///////
	fV = fSubImage[1][1];
	mRP[0][0]=mRP[0][0]+fV; mRP[2][0]=mRP[2][0]-fV; mRP[1][1]=mRP[1][1]-fV; mRP[3][1]=mRP[3][1]+fV; 
	mRP[0][2]=mRP[0][2]-fV; mRP[2][2]=mRP[2][2]+fV; mRP[1][3]=mRP[1][3]+fV; mRP[3][3]=mRP[3][3]-fV; 
	mIP[1][0]=mIP[1][0]-fV; mIP[3][0]=mIP[3][0]+fV; mIP[0][1]=mIP[0][1]-fV; mIP[2][1]=mIP[2][1]+fV; 
	mIP[1][2]=mIP[1][2]+fV; mIP[3][2]=mIP[3][2]-fV; mIP[0][3]=mIP[0][3]+fV; mIP[2][3]=mIP[2][3]-fV; 
	////////////////Row 1 Col 2///////
	fV = fSubImage[1][2];
	mRP[0][0]=mRP[0][0]+fV; mRP[2][0]=mRP[2][0]-fV; mRP[0][1]=mRP[0][1]-fV; mRP[2][1]=mRP[2][1]+fV; 
	mRP[0][2]=mRP[0][2]+fV; mRP[2][2]=mRP[2][2]-fV; mRP[0][3]=mRP[0][3]-fV; mRP[2][3]=mRP[2][3]+fV; 
	mIP[1][0]=mIP[1][0]-fV; mIP[3][0]=mIP[3][0]+fV; mIP[1][1]=mIP[1][1]+fV; mIP[3][1]=mIP[3][1]-fV; 
	mIP[1][2]=mIP[1][2]-fV; mIP[3][2]=mIP[3][2]+fV; mIP[1][3]=mIP[1][3]+fV; mIP[3][3]=mIP[3][3]-fV; 
	////////////////Row 1 Col 3///////
	fV = fSubImage[1][3];
	mRP[0][0]=mRP[0][0]+fV; mRP[2][0]=mRP[2][0]-fV; mRP[1][1]=mRP[1][1]+fV; mRP[3][1]=mRP[3][1]-fV; 
	mRP[0][2]=mRP[0][2]-fV; mRP[2][2]=mRP[2][2]+fV; mRP[1][3]=mRP[1][3]-fV; mRP[3][3]=mRP[3][3]+fV; 
	mIP[1][0]=mIP[1][0]-fV; mIP[3][0]=mIP[3][0]+fV; mIP[0][1]=mIP[0][1]+fV; mIP[2][1]=mIP[2][1]-fV; 
	mIP[1][2]=mIP[1][2]+fV; mIP[3][2]=mIP[3][2]-fV; mIP[0][3]=mIP[0][3]-fV; mIP[2][3]=mIP[2][3]+fV; 
	////////////////Row 2 Col 0///////
	fV = fSubImage[2][0];
	mRP[0][0]=mRP[0][0]+fV; mRP[1][0]=mRP[1][0]-fV; mRP[2][0]=mRP[2][0]+fV; mRP[3][0]=mRP[3][0]-fV; 
	mRP[0][1]=mRP[0][1]+fV; mRP[1][1]=mRP[1][1]-fV; mRP[2][1]=mRP[2][1]+fV; mRP[3][1]=mRP[3][1]-fV; 
	mRP[0][2]=mRP[0][2]+fV; mRP[1][2]=mRP[1][2]-fV; mRP[2][2]=mRP[2][2]+fV; mRP[3][2]=mRP[3][2]-fV; 
	mRP[0][3]=mRP[0][3]+fV; mRP[1][3]=mRP[1][3]-fV; mRP[2][3]=mRP[2][3]+fV; mRP[3][3]=mRP[3][3]-fV; 
	////////////////Row 2 Col 1///////
	fV = fSubImage[2][1];
	mRP[0][0]=mRP[0][0]+fV; mRP[1][0]=mRP[1][0]-fV; mRP[2][0]=mRP[2][0]+fV; mRP[3][0]=mRP[3][0]-fV; 
	mRP[0][2]=mRP[0][2]-fV; mRP[1][2]=mRP[1][2]+fV; mRP[2][2]=mRP[2][2]-fV; mRP[3][2]=mRP[3][2]+fV; 
	mIP[0][1]=mIP[0][1]-fV; mIP[1][1]=mIP[1][1]+fV; mIP[2][1]=mIP[2][1]-fV; mIP[3][1]=mIP[3][1]+fV; 
	mIP[0][3]=mIP[0][3]+fV; mIP[1][3]=mIP[1][3]-fV; mIP[2][3]=mIP[2][3]+fV; mIP[3][3]=mIP[3][3]-fV; 
	////////////////Row 2 Col 2///////
	fV = fSubImage[2][2];
	mRP[0][0]=mRP[0][0]+fV; mRP[1][0]=mRP[1][0]-fV; mRP[2][0]=mRP[2][0]+fV; mRP[3][0]=mRP[3][0]-fV; 
	mRP[0][1]=mRP[0][1]-fV; mRP[1][1]=mRP[1][1]+fV; mRP[2][1]=mRP[2][1]-fV; mRP[3][1]=mRP[3][1]+fV; 
	mRP[0][2]=mRP[0][2]+fV; mRP[1][2]=mRP[1][2]-fV; mRP[2][2]=mRP[2][2]+fV; mRP[3][2]=mRP[3][2]-fV; 
	mRP[0][3]=mRP[0][3]-fV; mRP[1][3]=mRP[1][3]+fV; mRP[2][3]=mRP[2][3]-fV; mRP[3][3]=mRP[3][3]+fV; 
	////////////////Row 2 Col 3///////
	fV = fSubImage[2][3];
	mRP[0][0]=mRP[0][0]+fV; mRP[1][0]=mRP[1][0]-fV; mRP[2][0]=mRP[2][0]+fV; mRP[3][0]=mRP[3][0]-fV; 
	mRP[0][2]=mRP[0][2]-fV; mRP[1][2]=mRP[1][2]+fV; mRP[2][2]=mRP[2][2]-fV; mRP[3][2]=mRP[3][2]+fV; 
	mIP[0][1]=mIP[0][1]+fV; mIP[1][1]=mIP[1][1]-fV; mIP[2][1]=mIP[2][1]+fV; mIP[3][1]=mIP[3][1]-fV; 
	mIP[0][3]=mIP[0][3]-fV; mIP[1][3]=mIP[1][3]+fV; mIP[2][3]=mIP[2][3]-fV; mIP[3][3]=mIP[3][3]+fV; 
	////////////////Row 3 Col 0///////
	fV = fSubImage[3][0];
	mRP[0][0]=mRP[0][0]+fV; mRP[2][0]=mRP[2][0]-fV; mRP[0][1]=mRP[0][1]+fV; mRP[2][1]=mRP[2][1]-fV; 
	mRP[0][2]=mRP[0][2]+fV; mRP[2][2]=mRP[2][2]-fV; mRP[0][3]=mRP[0][3]+fV; mRP[2][3]=mRP[2][3]-fV; 
	mIP[1][0]=mIP[1][0]+fV; mIP[3][0]=mIP[3][0]-fV; mIP[1][1]=mIP[1][1]+fV; mIP[3][1]=mIP[3][1]-fV; 
	mIP[1][2]=mIP[1][2]+fV; mIP[3][2]=mIP[3][2]-fV; mIP[1][3]=mIP[1][3]+fV; mIP[3][3]=mIP[3][3]-fV; 
	////////////////Row 3 Col 1///////
	fV = fSubImage[3][1];
	mRP[0][0]=mRP[0][0]+fV; mRP[2][0]=mRP[2][0]-fV; mRP[1][1]=mRP[1][1]+fV; mRP[3][1]=mRP[3][1]-fV; 
	mRP[0][2]=mRP[0][2]-fV; mRP[2][2]=mRP[2][2]+fV; mRP[1][3]=mRP[1][3]-fV; mRP[3][3]=mRP[3][3]+fV; 
	mIP[1][0]=mIP[1][0]+fV; mIP[3][0]=mIP[3][0]-fV; mIP[0][1]=mIP[0][1]-fV; mIP[2][1]=mIP[2][1]+fV; 
	mIP[1][2]=mIP[1][2]-fV; mIP[3][2]=mIP[3][2]+fV; mIP[0][3]=mIP[0][3]+fV; mIP[2][3]=mIP[2][3]-fV; 
	////////////////Row 3 Col 2///////
	fV = fSubImage[3][2];
	mRP[0][0]=mRP[0][0]+fV; mRP[2][0]=mRP[2][0]-fV; mRP[0][1]=mRP[0][1]-fV; mRP[2][1]=mRP[2][1]+fV; 
	mRP[0][2]=mRP[0][2]+fV; mRP[2][2]=mRP[2][2]-fV; mRP[0][3]=mRP[0][3]-fV; mRP[2][3]=mRP[2][3]+fV; 
	mIP[1][0]=mIP[1][0]+fV; mIP[3][0]=mIP[3][0]-fV; mIP[1][1]=mIP[1][1]-fV; mIP[3][1]=mIP[3][1]+fV; 
	mIP[1][2]=mIP[1][2]+fV; mIP[3][2]=mIP[3][2]-fV; mIP[1][3]=mIP[1][3]-fV; mIP[3][3]=mIP[3][3]+fV; 
	////////////////Row 3 Col 3///////
	fV = fSubImage[3][3];
	mRP[0][0]=mRP[0][0]+fV; mRP[2][0]=mRP[2][0]-fV; mRP[1][1]=mRP[1][1]-fV; mRP[3][1]=mRP[3][1]+fV; 
	mRP[0][2]=mRP[0][2]-fV; mRP[2][2]=mRP[2][2]+fV; mRP[1][3]=mRP[1][3]+fV; mRP[3][3]=mRP[3][3]-fV; 
	mIP[1][0]=mIP[1][0]+fV; mIP[3][0]=mIP[3][0]-fV; mIP[0][1]=mIP[0][1]+fV; mIP[2][1]=mIP[2][1]-fV; 
	mIP[1][2]=mIP[1][2]-fV; mIP[3][2]=mIP[3][2]+fV; mIP[0][3]=mIP[0][3]-fV; mIP[2][3]=mIP[2][3]+fV;

	//Finally take the magnitude...
	for(r=0;r<4;r++)
	{
		for(c=0;c<4;c++)
		{
			fSubImage[r][c]=(float)pow((double)(mRP[r][c]*mRP[r][c]+mIP[r][c]*mIP[r][c]),(double)0.5);
		}
	}

	//Unique values are 10: (0,0)-(2,2) and also (3,1)
	float* p = fUniqueVector;
	*p=fSubImage[0][0]/10.0; p++;
	*p=fSubImage[1][0]/10.0; p++;
	*p=fSubImage[2][0]/10.0; p++;

	*p=fSubImage[0][1]/10.0; p++;
	*p=fSubImage[1][1]/10.0; p++;
	*p=fSubImage[2][1]/10.0; p++;

	*p=fSubImage[0][2]/10.0; p++;
	*p=fSubImage[1][2]/10.0; p++;
	*p=fSubImage[2][2]/10.0; p++;

	*p=fSubImage[3][1]/10.0;
}
