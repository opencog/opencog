#ifndef DESTIN_DATA_H
#define DESTIN_DATA_H

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
        float* dImage;
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
         * Must be a path to the MNIST training data. See http://yann.lecun.com/exdb/mnist/
         * for an explanation of the data format.
         *
         * It loads the images into the mImagePointer vector
         *
         * @param sFileName const char* location of the file to be loaded.
         */
        void LoadFile(const char* sFileName);

        /**
         * Returns the pointer of the shifted image located on the GPU
         * @return float* pointer of shifted image
         */
        float* GetPointerDeviceImage(){ return dImage; };

        /**
         * Puts the selected image on the GPU with the shift applied to it
         * @param ImageIndex int index of the image loaded
         * @param RowShift int how many rows you want to shift the image
         * @param ColShift int how many cols you want to shift the image
         */
        void SetShiftedDeviceImage(int ImageIndex, int RowShift, int ColShift, int DemRow, int DemCol);

        /**
         * Returns the index of requested image
         * @param int iIndexOfImage
         * @return int label of requested index of image
         */
        int GetLabel(int iIndexOfImage){ return mLabels[iIndexOfImage]; };

        /**
         * Returns the number of images loaded
         * @return int number of images
         */
        int GetNumberOfImages(){ return mLabels.size(); };

        /**
         * Returns the number of unique labels loaded
         * @return int unique labels
         */
        int GetNumberOfUniqueLabels(){ return mUniqueLabels.size(); };

        /**
         * Get the list of labels
         * @param vector for int (Will be cleared)
         * @return the vector filled with the labels
         */
        void GetLabelList(vector<int>& vLabels);

        /**
         * Get the unique list of labels
         * @param vector for int (Will be cleared)
         * @return the vector filled with the unique labels
         */
        void GetUniqueLabels(vector<int>& vUniqueLabels);

        /**
         * Get the index numbers of a certain label
         * @param int label where you want the index of
         * @param vector int for index of label
         * @return vector int with index of requested label
         */
        void GetIndicesForThisLabel(int Label, vector<int>& IndicesForThisLabel );

        /**
         * Very big function from original DeSTIN to do FFT on 4x4
         * @param float** sub image
         * @param float* vector
         */
        void DoSpecial4x4FFT(float** &fSubImage, float* fUniqueVector);
};

#endif
