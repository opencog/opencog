#ifndef DESTIN_LAYER_H
#define DESTIN_LAYER_H

#include <iostream>
#include <fstream>
//#include "DestinNode.h"

using namespace std;

class DestinLayerLatch
{
private:
	int mRows;
	int mCols;
	//vector< DestinNodeLatchData* > mDestinNodeLatchUnits;
	int mGTLabel;
	int mSignalIndex;
	int mMovementNumber;
	int mObservationNumber;
	bool mbReset;

public:
	DestinLayerLatch(void);

	~DestinLayerLatch(void);

	void Create(int Rows, int Cols, int States );

	//DestinNodeLatchData* GetPointerToNode(int r, int c);

	void SetDiagnosticData(int GTLabel, int SignalIndex, int MovementNumber, int ObservationNumber);

	// inline to get the diag data
	int GetGroundTruthLabel() { return mGTLabel; };

	int GetSignalIndex() { return mSignalIndex; };

	int GetMovementNumber() { return mMovementNumber; };

	int GetObservationNumber() { return mObservationNumber; };

	bool GetReset(){ return mbReset; };

//	void DisplayDiagnosticInformation(ostream& stm)
//	{
//		stm << "GTLabel," << mGTLabel << endl;
//		stm << "SignalIndex," << mSignalIndex << endl;
//		stm << "MovementNumber," << mMovementNumber << endl;
//		stm << "ObservationNumber," << mObservationNumber << endl;
//	};
};

class DestinLayer
{
private:
    int mRows;
	int mCols;
	//vector<DestinNode*> mDestinNodeUnits;
	int mGTLabel;
	int mSignalIndex;
	int mMovementNumber;
	int mObservationNumber;
	int mMaxChildrenPerNode;
        
public:
	DestinLayer(void);

	~DestinLayer(void);

	void ClearAndDestroy();

	/**
	 * Create Layer
	 * @param Rows number of rows
	 * @param Cols number of columns
	 * @param States number of states
	 * @param ParentStates amount of states of parent
	 * @param InputDimensionality
	 * @param inputDimensionalities[] if this is NULL, then InputDimensionality is used for all the nodes, otherwise, use this array of inputDimensinality to assign each node its own.
	 * @param DType distance method
	 * @param bBinaryPOS
	 * @param bAveraging
	 * @param bUseStarvationCoefficient
	 * @param PSSAUpdateDelay
	 * @param bIgnoreAdvice
	 * @param dcMu
	 * @param dcSigma
	 * @param dcRho
	 * @param bUseDecayLearningRate
	 * @param iDecayKickInPoint
	 * @param fRhoThreshold
	 * @param bUseRhoDerivative
	 * @param bConstrainInitialCentroids
	 * @param iBlocksToProcess
	 * @param iLayerNumber
	 * @param iMovementsForCluster
	 * @param BasicOnlineClustering
	 * @param FixedRate
	 * @param bUseGoodPOS
	 * @param SequenceLength
	 * @param bTopNode
	 */
	void Create(int Rows, int Cols,	int States, int ParentStates, int InputDimensionality, int inputDimensionalities[],
	            int DType, bool bBinaryPOS, bool bAveraging, bool bUseStarvationCoefficient, int PSSAUpdateDelay,
	            bool bIgnoreAdvice, double dcMu, double dcSigma, double dcRho, bool bUseDecayLearningRate,
	            int iDecayKickInPoint, float fRhoThreshold, bool bUseRhoDerivative, bool bConstrainInitialCentroids,
	            int iBlocksToProcess, int iLayerNumber, int iMovementsForCluster, bool BasicOnlineClustering,
	            float FixedRate, bool bUseGoodPOS, int SequenceLength, bool bTopNode);

	int GetNumberOfRows(){ return mRows;}

	int GetNumberOfCols(){ return mCols;}

	//int GetNumberOfCentroids(){ return mDestinNodeUnits[0]->GetNumberOfStates(); };

	void AssignChildrenAndParents(int Layer,int NumberOfLayers, bool bInitialLayerIsTransformative); //uses the 'default' 4:1 method unless bInitialLayerIsTransformative is true
};

#endif
