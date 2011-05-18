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
	mShiftedImageCache=NULL;
}

DestinData::~DestinData(void)
{
	if ( mShiftedImageCache != NULL ) 
	{
		for(int r=0;r<mRows;r++)
		{
			delete mShiftedImageCache[r];
		}
		delete mShiftedImageCache;
	}
}

int DestinData::GetLabel(int iIndexOfImage)
{
	return mLabels[iIndexOfImage];
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
	cout << "Finished reading file." << endl;
}

int DestinData::GetNumberOfImages()
{
	return mLabels.size();
}

int DestinData::GetNumberOfUniqueLabels()
{
	return mUniqueLabels.size();
}

void DestinData::GetLabelList(vector<int>& Labels)
{
	Labels.clear();
	for(int i=0;i<(int)(mLabels.size());i++)
	{
		Labels.push_back(mLabels[i]);
	}
}

void DestinData::GetShiftedImage(int ImageIndex, int RowShift, int ColShift, float** &fData )
{
    int R=4;
    int C=4;
    int size = mRows*mCols*sizeof(float);
    cout << "Single image is: " << size << " Bytes." << endl;
    cout << "Single image is: " << 256*256*sizeof(float) << " Bytes." << endl;
    //allocate memory if necessary...
    if ( fData==NULL )
    {
        fData = new float*[mRows];
        for (int rr=0;rr<mRows;rr++)
        {
            fData[rr]=new float[mCols];
        }
    }

    if ( ImageIndex!=mLastImageIndex )
    {
        //Load the image into the 50x50 buffer with the "0,0" offset...
        for(int r=0;r<50;r++)
        {
            for(int c=0;c<50;c++)
            {
                mImageWithOffset[r][c]=0;
            }
        }
        float** fImage = mImagePointer[ImageIndex];
        for(int r=0;r<mRows;r++)
        {
            for(int c=0;c<mCols;c++)
            {
                mImageWithOffset[r+R][c+C]=fImage[r][c];
            }
        }
    }

    //Now load the data using the offset provided...
    for(int r=0;r<mRows;r++)
    {
        for(int c=0;c<mCols;c++)
        {
            fData[r][c]=mImageWithOffset[r+RowShift][c+ColShift];
        }
    }
    mLastImageIndex=ImageIndex;
}

void DestinData::GetSubImage(int ImageIndex,int RowShift,int ColShift,
		int rS, int rE,int cS,int cE,float** &fSubImage)
{
	GetShiftedImage(ImageIndex,RowShift,ColShift,mShiftedImageCache);
	int nRows=rE-rS+1;
	int nCols=cE-cS+1;
	float* fRowOut;
	float* fShiftedImageCacheRow;
	for(int r=0;r<nRows;r++)
	{
		fRowOut = fSubImage[r];
		fShiftedImageCacheRow=mShiftedImageCache[r+rS];
		for(int c=0;c<nCols;c++)
		{
			*(fRowOut+c)=fShiftedImageCacheRow[c+cS];	
		}
	}
}

//note: this only does a 4x4 FFT and returns all 16 coefficients as magnitude in fSubImage
void DestinData::GetSubImageFFT(int ImageIndex,int RowShift,int ColShift,
        int rS, int rE,int cS,int cE,float** &fSubImage, float* fUniqueVector)
{
    GetSubImage(ImageIndex,RowShift,ColShift,rS,rE,cS,cE,fSubImage);
    DoSpecial4x4FFT(fSubImage,fUniqueVector);
}

void DestinData::GetSubImageVector(int ImageIndex,int RowShift,int ColShift,
		int rS, int rE,int cS,int cE,float** &fSubImage, float* fVector)
{
	GetShiftedImage(ImageIndex,RowShift,ColShift,mShiftedImageCache);
	int nRows=rE-rS+1;
	int nCols=cE-cS+1;
	float* fRowOut;
	float* fShiftedImageCacheRow;
	int kj=0;
	for(int r=0;r<nRows;r++)
	{
		fRowOut = fSubImage[r];
		fShiftedImageCacheRow=mShiftedImageCache[r+rS];
		for(int c=0;c<nCols;c++)
		{
			*(fRowOut+c)=fShiftedImageCacheRow[c+cS];	
			*(fVector+kj)=fShiftedImageCacheRow[c+cS];
			kj++;
		}
	}
}

void DestinData::WriteToCSV(int ImageIndex, int RowShift, int ColShift, char* cFile)
{
	float** fData = NULL;
	GetShiftedImage(ImageIndex, RowShift, ColShift, fData );
	std::ofstream stmCSV;
	stmCSV.open(cFile,ios::out);
	float* fRow;
	for(int r=0;r<mRows;r++)
	{
		fRow = fData[r];
		for(int c=0;c<mCols;c++)
		{
			stmCSV << *(fRow+c);
			if ( c != mCols-1 )
			{
				stmCSV << ",";
			}
		}
		stmCSV << endl;
	}
	stmCSV.close();
}

void DestinData::GetUniqueLabels(vector<int>& vLabels)
{
	vLabels.clear();
	vector<int>::iterator it = this->mUniqueLabels.begin();
	while ( it != mUniqueLabels.end() )
	{
		vLabels.push_back(*it++);
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
