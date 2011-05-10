#include "DestinLayer.h"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <stdexcept>

using namespace std;
using std::tr1::shared_ptr;

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

//This assumes that the csv files are in this directory and have a convention:
// Layer01_Row_03_Col_01_CENTROIDS.csv
// Layer01_Row_03_Col_01_SMAX.csv
void DestinLayer::OverrideCentroidsAndSMaxWithCSVFiles(int Layer, char* sDirectory)
{
	char cCentroidFileName[128];
	char cSMAXFileName[128];

	int r,c;
	for(r=0;r<mRows;r++)
	{
		for(c=0;c<mCols;c++)
		{
			sprintf(cCentroidFileName,"%s/Layer%.2d_Row_%.2d_Col_%.2d_CENTROIDS.csv",sDirectory,Layer,r,c);			
			sprintf(cSMAXFileName,"%s/Layer%.2d_Row_%.2d_Col_%.2d_SMAX.csv",sDirectory,Layer,r,c);	
//			mDestinNodeUnits[c+r*mCols]->OverrideCentroidsWithCSV(cCentroidFileName);
//			mDestinNodeUnits[c+r*mCols]->OverrideSMaxWithCSV(cSMAXFileName);
		}
	}
}

//Write to a stream...
bool DestinLayer::WriteToStream(std::ofstream& stmOutput)
{
	stmOutput.write( (char*)&mRows, sizeof(mRows) );
	stmOutput.write( (char*)&mCols, sizeof(mCols) );

	stmOutput.write( (char*)&mGTLabel, sizeof(mGTLabel) );
	stmOutput.write( (char*)&mSignalIndex, sizeof(mSignalIndex) );
	stmOutput.write( (char*)&mMovementNumber, sizeof(mMovementNumber) );
	stmOutput.write( (char*)&mObservationNumber, sizeof(mObservationNumber) );

//	DestinNode* MyNode;
//	int r,c;
//	for(r=0;r<mRows;r++)
//	{
//		for(c=0;c<mCols;c++)
//		{
//			MyNode = this->GetPointerToNode(r,c);
//			MyNode->WriteToStream(stmOutput);
//		}
//	}

	return true;
}

// Read from a stream...
bool DestinLayer::ReadFromStream(std::ifstream& stmInput)
{
	stmInput.read( (char*)&mRows, sizeof(mRows) );
	stmInput.read( (char*)&mCols, sizeof(mCols) );

	stmInput.read( (char*)&mGTLabel, sizeof(mGTLabel) );
	stmInput.read( (char*)&mSignalIndex, sizeof(mSignalIndex) );
	stmInput.read( (char*)&mMovementNumber, sizeof(mMovementNumber) );
	stmInput.read( (char*)&mObservationNumber, sizeof(mObservationNumber) );

//	DestinNode* MyNode;
//	int r,c;
//	for (r=0;r<mDestinNodeUnits.size();r++)
//	{
//		delete mDestinNodeUnits[r];
//	}
//	mDestinNodeUnits.clear();
//	for(r=0;r<mRows;r++)
//	{
//		for(c=0;c<mCols;c++)
//		{
//			MyNode = new DestinNode();
//			MyNode->ReadFromStream(stmInput);
//			mDestinNodeUnits.push_back(MyNode);
//		}
//	}

	return true;
}


bool DestinLayer::operator == (DestinLayer& o)
{
	bool bReturn = true;
	if ( mRows != o.mRows ) bReturn = false;
	if ( mCols != o.mCols ) bReturn = false;
	if ( mGTLabel != o.mGTLabel ) bReturn = false;
	if ( mSignalIndex != o.mSignalIndex ) bReturn = false;
	if ( mMovementNumber != o.mMovementNumber ) bReturn = false;
	if ( mObservationNumber != o.mObservationNumber ) bReturn = false;

//	DestinNode* MyNode;
//	DestinNode* ItsNode;
//
//	if ( bReturn )
//	{
//		int r,c;
//		for(r=0;r<mRows;r++)
//		{
//			for(c=0;c<mCols;c++)
//			{
//				MyNode = this->GetPointerToNode(r,c);
//				ItsNode = o.GetPointerToNode(r,c);
//				if ( !(*MyNode==*ItsNode) )
//				{
//					bool bTest = (*MyNode==*ItsNode);
//					bReturn = false;
//				}
//			}
//		}
//	}


	return bReturn;
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
//			iNodeID = c+1000*r+LayerNumber*10000;
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
void DestinLayerLatch::SetDiagnosticData(int GTLabel, int SignalIndex, int MovementNumber, int ObservationNumber)
{
	mGTLabel=GTLabel;
	mSignalIndex=SignalIndex;
	mMovementNumber=MovementNumber;
	mObservationNumber=ObservationNumber;

}

void DestinLayer::SetDiagnosticData(int GTLabel, int SignalIndex, int MovementNumber, int ObservationNumber)
{
	mGTLabel=GTLabel;
	mSignalIndex=SignalIndex;
	mMovementNumber=MovementNumber;
	mObservationNumber=ObservationNumber;
}

void DestinLayer::ClearValidOutputFlag()
{
//	for(int r=0;r<mRows;r++)
//	{
//		for(int c=0;c<mCols;c++)
//		{
//			mDestinNodeUnits[c+r*mCols]->ClearValidOutputFlag(); // put relevant data in latch...
//		}
//	}
}

void DestinLayer::SetNextUpdateCountAndSequenceLength(int UC, int SL)
{
//	for(int r=0;r<mRows;r++)
//	{
//		for(int c=0;c<mCols;c++)
//		{
//			mDestinNodeUnits[c+r*mCols]->SetNextUpdateCountAndSequenceLength(UC,SL);
//		}
//	}
}

void DestinLayer::SetCompileCentroidShiftMetrics(bool b)
{
//	for(int r=0;r<mRows;r++)
//	{
//		for(int c=0;c<mCols;c++)
//		{
//			mDestinNodeUnits[c+r*mCols]->SetCompileCentroidShiftMetrics(b);
//		}
//	}
}

void DestinLayer::LatchData(DestinLayerLatch& oLatch)
{
//	DestinNodeLatchData* LatchData;
//	for(int r=0;r<mRows;r++)
//	{
//		for(int c=0;c<mCols;c++)
//		{
//			LatchData=oLatch.GetPointerToNode(r,c); //get pointer to nodes latch data...
//			mDestinNodeUnits[c+r*mCols]->GetDataForLatching(*LatchData); // put relevant data in latch...
//			// For feedback, you can use LatchData
//		}
//	}
//	//set the diagnostic data too...
//	oLatch.SetDiagnosticData(mGTLabel,mSignalIndex,mMovementNumber,mObservationNumber);
}


//DestinNode* DestinLayer::GetPointerToNode(int r, int c)
//{
//	return mDestinNodeUnits[c+r*mCols];
//}

DestinLayerLatch::DestinLayerLatch(void)
{
	mRows=0;
	mCols=0;
	mGTLabel=-1;	//ground truth label, i.e, "letter A", "digit B", "Dog"
	mSignalIndex=-1; //index, for example if we have 100 things in the data set this could be 0-99
	mMovementNumber=-1; // which movement we are on
	mObservationNumber=-1; //which observation we are working on iteratively

}

DestinLayerLatch::~DestinLayerLatch(void)
{
//	for(int r=0;r<(int)(mDestinNodeLatchUnits.size());r++)
//	{
//		delete mDestinNodeLatchUnits[r];
//	}
}
//DestinNodeLatchData* DestinLayerLatch::GetPointerToNode(int r, int c)
//{
//	return mDestinNodeLatchUnits[c+r*mCols];
//}

void DestinLayerLatch::Create(int Rows, int Cols, int States )
{
    DestinLayerLatch();
	mRows=Rows;
	mCols=Cols;
	//mDestinNodeLatchUnits.clear();

//	DestinNodeLatchData* p;// = mDestinNodeLatchUnits;
//	for(int r=0;r<mRows;r++)
//	{
//		for(int c=0;c<mCols;c++)
//		{
//			p=new DestinNodeLatchData();
//			p->Create(States);
//			mDestinNodeLatchUnits.push_back(p);
//		}
//	}
}

DestinLayer::FamilySizes DestinLayer::calcFamilySizes(int parentLayerNodeCount, int childNodeCount) {
	const int nn = parentLayerNodeCount;
	FamilySizes fs;

	fs.nLargeFamilies = 0;
	fs.smallFamilySize = 0;
	fs.largeFamilySize = 0;
	fs.nSmallFamilies = 0;
	if (parentLayerNodeCount != 0) {
		fs.nLargeFamilies = childNodeCount % nn;
		fs.smallFamilySize = childNodeCount / nn;
		fs.largeFamilySize = fs.smallFamilySize + 1;
		fs.nSmallFamilies = nn - fs.nLargeFamilies;

		if(fs.nLargeFamilies == 0){
			//if there are only small familyies, make them the big families,
			//so thay there are always big families
			fs.nLargeFamilies = fs.nSmallFamilies;
			fs.largeFamilySize = fs.smallFamilySize;
			fs.smallFamilySize = 0;
			fs.nSmallFamilies = 0;
		}
	}
	return fs;
}

void DestinLayer::AssignChildrenAndParents(int childLayerRows, int childLayerCols, int parentLayerRows, int parentLayerCols){
    const int nParentNodes = parentLayerRows * parentLayerCols;
    const int nChildNodes = childLayerCols * childLayerRows;
    //const int nn=mDestinNodeUnits.size();//number of nodes
    const int nn=0;//number of nodes

    if(nn<1){
        throw logic_error("Can't assign children and parents for a layer with no nodes.");
    }

    if(nChildNodes!=0 && nn > nChildNodes){
        throw invalid_argument("This layer can't have more nodes than its child layer.");
    }

    if(nParentNodes!=0 && nParentNodes > nn ){
        throw invalid_argument("Parent layer can't have more nodes than this layer.");
    }

    int r, c;

	FamilySizes fs = calcFamilySizes(nParentNodes,nn);

    //assign parent nodes
    for(int pn = 0 ; pn < fs.nLargeFamilies ; pn ++){
        for(int cn = 0 ; cn  < fs.largeFamilySize ; cn ++){
            r = pn / parentLayerCols;
            c = pn % parentLayerCols;
            //mDestinNodeUnits.at(pn * fs.largeFamilySize + cn)->AddParentNode(r,c);
        }
    }
    int offset  = fs.nLargeFamilies * fs.largeFamilySize;
    for(int pn = 0 ; pn < fs.nSmallFamilies ; pn ++){
        for(int cn = 0 ; cn < fs.smallFamilySize ; cn++){
            r = (pn + fs.nLargeFamilies) / parentLayerCols;
            c = (pn + fs.nLargeFamilies) % parentLayerCols;
            //mDestinNodeUnits.at(pn * fs.smallFamilySize + cn + offset)->AddParentNode(r,c);
        }
    }

	fs = calcFamilySizes(nn,nChildNodes);

	//assign child nodes
	
    for(int pn = 0 ; pn < fs.nLargeFamilies ; pn++){
        for(int cn = 0 ; cn < fs.largeFamilySize ; cn++){
            r = (pn * fs.largeFamilySize + cn) / childLayerCols;
            c = (pn * fs.largeFamilySize + cn) % childLayerCols;
            //mDestinNodeUnits.at(pn )->AddChildNode(r,c);
        }
    }
    offset  = fs.nLargeFamilies * fs.largeFamilySize;

    for(int pn = 0 ; pn < fs.nSmallFamilies ; pn++){
        for(int cn = 0 ; cn< fs.smallFamilySize ; cn++){
            int index = pn * fs.smallFamilySize + cn + offset;
            r = index / childLayerCols;
            c = index % childLayerCols;
            //mDestinNodeUnits.at(pn + fs.nLargeFamilies)->AddChildNode(r,c);
        }
    }
    mMaxChildrenPerNode = fs.largeFamilySize;
	return;
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
				}	//c
			}	//r
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
						}	//ChildCol
					}	//ChildRow
				}	//c
			}	//r
		}	//Layer

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
					}	//c
				}	//r	
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
					}	//c
				}	//r		
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
						}	//ChildCol
					}	//ChildRow

				}	//c
			}	//r
		}	//Layer

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
				}	//c
			}	//r		
		}
	}

}

