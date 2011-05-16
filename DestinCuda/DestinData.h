#pragma once

#include <vector>
#include <map>
using namespace std;

class DestinData
{
    private:
        vector<float**> mImagePointer;
        vector<int> mLabels;
        map< int, vector<int> > mMapLabelToIndexVector;
        vector<int> mUniqueLabels;

        int mLastImageIndex;
        float mImageWithOffset[50][50];
        int mRows;
        int mCols;
        float** mShiftedImageCache;

        //scratch for real & image part of FFT
        float mRP[4][4];
        float mIP[4][4];

public:
	DestinData(void);
	~DestinData(void);
	void LoadFile(const char* sFileName);
	int GetNumberOfImages();
	int GetNumberOfUniqueLabels();
	void GetLabelList(vector<int>& Labels);
	void GetShiftedImage(int ImageIndex,int RowShift, int ColShift, float** &fData);
	void WriteToCSV(int ImageIndex, int RowShift, int ColShift, char* cFile );
	void GetUniqueLabels(vector<int>& UniqueLabels);
	void GetIndicesForThisLabel(int Label, vector<int>& IndicesForThisLabel );
	void GetSubImage(int ImageIndex,int RowShift,int ColShift,
		int rS, int rE,int cS,int cE,float** &fSubImage);
	void GetSubImageVector(int ImageIndex,int RowShift,int ColShift,
		int rS, int rE,int cS,int cE,float** &fSubImage, float* fVector);
	void GetSubImageFFT(int ImageIndex,int RowShift,int ColShift,
		int rS, int rE,int cS,int cE,float** &fSubImage, float* fUniqueVector);
	void DoSpecial4x4FFT(float** &fSubImage, float* fUniqueVector);
	int GetLabel(int iIndexOfImage);

};
