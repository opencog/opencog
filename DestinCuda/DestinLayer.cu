#include "DestinLayer.h"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <stdexcept>

using namespace std;

DestinLayer::DestinLayer(void)
{
	mRows=0;
	mCols=0;
	mGTLabel=-1;	//ground truth label, i.e, "letter A", "digit B", "Dog"
	mSignalIndex=-1; //index, for example if we have 100 things in the data set this could be 0-99
	mMovementNumber=-1; // which movement we are on
	mObservationNumber=-1; //which observation we are working on iteratively
    mMaxChildrenPerNode=0; //after AssignChildrenAndParents(int,int,int,int) is called, this stores
                            //the maximum number of children any of these nodes has.
}

DestinLayer::~DestinLayer(void)
{
	ClearAndDestroy();
}

void DestinLayer::ClearAndDestroy(void)
{
//	for(int r=0;r<(int)(mDestinNodeUnits.size());r++)
//	{
//		delete mDestinNodeUnits[r];
//	}
//	mDestinNodeUnits.clear();
}

void DestinLayer::Create( int Rows, int Cols, int States, int ParentStates, int InputDimensionality, int inputDimensionalities[],
                          int DType, bool bBinaryPOS, bool bAveraging, bool bUseStarvationCoefficient, int PSSAUpdateDelay,
                          bool bIgnoreAdvice, double dcMu, double dcSigma, double dcRho, bool bUseDecayingLearningRate,
                          int iDecayKickInPoint, float fRhoThresholdPoint, bool bUseRhoDerivative, bool bConstrainInitialCentroids,
                          int iBlocksToProcess,	int LayerNumber, int iMovementsForCluster, bool BasicOnlineClustering,
                          float FixedRate, bool bDoGoodPOS, int SequenceLength, bool bTopNode )
{
    DestinLayer();
	mRows=Rows;
	mCols=Cols;
//	mDestinNodeUnits.clear();
//	DestinNode* p;
//
//	int iNodeID;
//	for(int r=0;r<mRows;r++)
//	{
//		for(int c=0;c<mCols;c++)
//		{
//			if(inputDimensionalities!=NULL){
//				InputDimensionality = inputDimensionalities[r*mCols + c];
//			}
//			iNodeID = c+1000*r+10000*LayerNumber;
//			p=new DestinNode();
//			p->Create(States,ParentStates,InputDimensionality,
//				DType,
//				bBinaryPOS,
//				bAveraging,
//				bUseStarvationCoefficient,
//				bIgnoreAdvice,
//				dcMu, dcSigma, dcRho,
//				bUseDecayingLearningRate,
//				iDecayKickInPoint,
//				fRhoThresholdPoint,
//				bUseRhoDerivative,
//				bConstrainInitialCentroids,
//				iBlocksToProcess,
//				iNodeID,
//				iMovementsForCluster,
//				BasicOnlineClustering,
//				FixedRate,r,c,LayerNumber, bDoGoodPOS, SequenceLength,bTopNode);
//
//			p->SetPSSAUpdateDelay(PSSAUpdateDelay);
//			mDestinNodeUnits.push_back(p);
//
//		}
//	}
}

//This assumes layer=0 is the sensory interface layer...with no children...
// ...and layer NumberOfLayers-1 is the top layer with no parents
//uses the 'default' 4:1 method UNLESS bUsesTransformationalLayer is true in which cases layers 1-up are 4:1 but 0-1 are 1:1
void DestinLayer::AssignChildrenAndParents(int Layer, int NumberOfLayers, bool bUsesTransformationalLayer)
{
    int parentR;
    int parentC;

    if ( bUsesTransformationalLayer )
    {
        //Assign the children of each node here assuming a 4x1 relationship, i.e., I have 4 children laid out in 2-d space...
        // except the first two layers which are 1:1
        if ( Layer==1 )
        {
            for (int r=0;r<mRows;r++)
            {
                for(int c=0;c<mCols;c++)
                {
                    //mDestinNodeUnits[c+r*mCols]->AddChildNode(r,c);
                }   //c
            }   //r
        }
        else if ( Layer >=2 )
        {
            for (int r=0;r<mRows;r++)
            {
                for(int c=0;c<mCols;c++)
                {
                    // to match MATLAB, add the child nodes by row first...
                    for (int ChildCol=2*c;ChildCol<=2*c+1;ChildCol++)
                    {
                        for (int ChildRow=2*r;ChildRow<=2*r+1;ChildRow++)
                        {
                            //The children here are ChildRow,ChildCol...
                            //mDestinNodeUnits[c+r*mCols]->AddChildNode(ChildRow,ChildCol);
                        }   //ChildCol
                    }   //ChildRow
                }   //c
            }   //r
        }   //Layer

        //Now assign the parent! This is a little odder...

        if ( Layer != NumberOfLayers-1 )
        {
            if ( Layer==0 )
            {
                for (int r=0;r<mRows;r++)
                {
                    for(int c=0;c<mCols;c++)
                    {
                        //The parents here are r c since this is 1:1
                        //mDestinNodeUnits[c+r*mCols]->AddParentNode(r,c);
                    }   //c
                }   //r
            }
            else
            {
                for (int r=0;r<mRows;r++)
                {
                    for(int c=0;c<mCols;c++)
                    {
                        //The parents here are floor(mRows/2),floor(mCols/2)
                        parentR=(int)floor((float)r/(float)2.0);
                        parentC=(int)floor((float)c/(float)2.0);
                        //mDestinNodeUnits[c+r*mCols]->AddParentNode(parentR,parentC);
                    }   //c
                }   //r
            }
        }
    }
    else
    {
        //Assign the children of each node here assuming a 4x1 relationship, i.e., I have 4 children laid out in 2-d space...
        if ( Layer != 0 )
        {
            for (int r=0;r<mRows;r++)
            {
                for(int c=0;c<mCols;c++)
                {
                    // to match MATLAB, add the child nodes by row first...
                    for (int ChildCol=2*c;ChildCol<=2*c+1;ChildCol++)
                    {
                        for (int ChildRow=2*r;ChildRow<=2*r+1;ChildRow++)
                        {
                            //The children here are ChildRow,ChildCol...
                            //mDestinNodeUnits[c+r*mCols]->AddChildNode(ChildRow,ChildCol);
                        }   //ChildCol
                    }   //ChildRow

                }   //c
            }   //r
        }   //Layer

        //Now assign the parent! This is a little odder...

        if ( Layer != NumberOfLayers-1 )
        {
            for (int r=0;r<mRows;r++)
            {
                for(int c=0;c<mCols;c++)
                {
                    //The parents here are floor(mRows/2),floor(mCols/2)
                    parentR=(int)floor((float)r/(float)2.0);
                    parentC=(int)floor((float)c/(float)2.0);
                    //mDestinNodeUnits[c+r*mCols]->AddParentNode(parentR,parentC);
                }   //c
            }   //r
        }
    }
}

DestinLayerLatch::DestinLayerLatch(void)
{
    mRows=0;
    mCols=0;
    mGTLabel=-1;    //ground truth label, i.e, "letter A", "digit B", "Dog"
    mSignalIndex=-1; //index, for example if we have 100 things in the data set this could be 0-99
    mMovementNumber=-1; // which movement we are on
    mObservationNumber=-1; //which observation we are working on iteratively

}

DestinLayerLatch::~DestinLayerLatch(void)
{
//  for(int r=0;r<(int)(mDestinNodeLatchUnits.size());r++)
//  {
//      delete mDestinNodeLatchUnits[r];
//  }
}
//DestinNodeLatchData* DestinLayerLatch::GetPointerToNode(int r, int c)
//{
//  return mDestinNodeLatchUnits[c+r*mCols];
//}

void DestinLayerLatch::Create(int Rows, int Cols, int States )
{
    DestinLayerLatch();
    mRows=Rows;
    mCols=Cols;
    //mDestinNodeLatchUnits.clear();

//  DestinNodeLatchData* p;// = mDestinNodeLatchUnits;
//  for(int r=0;r<mRows;r++)
//  {
//      for(int c=0;c<mCols;c++)
//      {
//          p=new DestinNodeLatchData();
//          p->Create(States);
//          mDestinNodeLatchUnits.push_back(p);
//      }
//  }
}
