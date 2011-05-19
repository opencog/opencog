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
        float mImageWithOffset[40][40];
        int mRows;
        int mCols;
        float* dmImage;
        float *mImage;

        //scratch for real & image part of FFT
        float mRP[4][4];
        float mIP[4][4];

    public:
        /**
         * Constructor of DestinData
         */
        DestinData(void);

        /**
         * Deconstructor of DestinData
         */
        ~DestinData(void);

        /**
         * Load the data inside DestinData
         * @param sFileName const char* location of the file to be loaded.
         */
        void LoadFile(const char* sFileName);

        /**
         * Returns the pointer of the shifted image located on the GPU
         * @return float* pointer of shifted image
         */
        float* GetPointerDeviceImage(){ return dmImage; };

        /**
         * Puts the selected image on the GPU with the shift applayed to it
         * @param ImageIndex int index of the image loaded
         * @param RowShift int how many rows you want to shift the image
         * @param ColShift int home many cols you want to shift the image
         */
        void SetShiftedDeviceImage(int ImageIndex, int RowShift, int ColShift);

        int GetLabel(int iIndexOfImage){ return mLabels[iIndexOfImage]; };
        int GetNumberOfImages(){ return mLabels.size(); };
        int GetNumberOfUniqueLabels(){ return mUniqueLabels.size(); };
        void GetLabelList(vector<int>& Labels);
        void GetUniqueLabels(vector<int>& UniqueLabels);
        void GetIndicesForThisLabel(int Label, vector<int>& IndicesForThisLabel );
        void DoSpecial4x4FFT(float** &fSubImage, float* fUniqueVector);
};
