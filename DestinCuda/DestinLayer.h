#ifndef DESTIN_LAYER_H
#define DESTIN_LAYER_H
//#include "DestinNode.h"
#include <boost/tr1/memory.hpp>

using namespace std;
using std::tr1::shared_ptr;

class DestinLayerLatch
{
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

	//Write to a stream...
	bool WriteToStream(std::ofstream& stmOutput);

	// Read from a stream...
	bool ReadFromStream(std::ifstream& stmInput);

	void ClearValidOutputFlag();

	void SetNextUpdateCountAndSequenceLength(int UC, int SL);

	void SetCompileCentroidShiftMetrics(bool b);

	void Create(int Rows, int Cols,
			int States, int ParentStates, 
			int InputDimensionality,
			int inputDimensionalities[], // if this is NULL, then use the same InputDimensionality for all the nodes,
										 // otherwise, use this array of inputDimensinality to assign each node its own.
			int DType,		//distance method
			bool bBinaryPOS,
			bool bAveraging,
			bool bUseStarvationCoefficient,
			int PSSAUpdateDelay,
			bool bIgnoreAdvice,
			double dcMu, 
			double dcSigma, 
			double dcRho,
			bool bUseDecayLearningRate, 
			int iDecayKickInPoint, 
			float fRhoThreshold,
			bool bUseRhoDerivative,
			bool bConstrainInitialCentroids,
			int iBlocksToProcess,
			int iLayerNumber,
			int iMovementsForCluster,
			bool BasicOnlineClustering,
			float FixedRate,
			bool bUseGoodPOS,
			int SequenceLength,
			bool bTopNode);

	void OverrideCentroidsAndSMaxWithCSVFiles(int Layer, char* sDirectory);

	int GetNumberOfRows(){ return mRows;}

	int GetNumberOfCols(){ return mCols;}

	//int GetNumberOfCentroids(){ return mDestinNodeUnits[0]->GetNumberOfStates(); };

	//DestinNode* GetPointerToNode(int r, int c);

	void LatchData(DestinLayerLatch& oLatch);

	void SetDiagnosticData(int GTLabel, int SignalIndex, int MovementNumber, int ObservationNumber);


    /**
     * AssignChildrenAndParents
     *
     * Doesn't assume a 4 to 1 heirachy, instead assigns the nodes in a linear fashion
     * instead of in a square like fashion. Depending on the size of the input layers,
     * some node in this layer may have a one less child than the rest, if there
     * are not enough children to go around.
     * For example, if there are two layers, child layer having 7 nodes and
     * parent layer haveing 5  nodes :
     *
     * child layer : 0123456
     * parent layer 2: 01234
     *
     * then children to parent relationships (a.k.a families) will be:
     * (0,1)->0 (2,3)->1, (4)->2, (5)->3, (6)->4
     * As you can see, the 2 larger "families" are created before the 3 smaller "families"
     *
     * If childLayerRows * childLayerCols = 0, because this layer is the bottom input layer,
     * then it doesnt assign an input layer, and simularly, if parentLayerRows*parentLayerCols = 0 then a parent layer
     * is not assigned, presumably because this layer is the top layer.
     * @param childLayerRows
     * @param childLayerCols
     * @param parentLayerRows
     * @param parentLayerCols
     *
     */
    void AssignChildrenAndParents(int childLayerRows, int childLayerCols, int parentLayerRows, int parentLayerCols);

	void AssignChildrenAndParents(int Layer,int NumberOfLayers, bool bInitialLayerIsTransformative); //uses the 'default' 4:1 method unless bInitialLayerIsTransformative is true

	bool operator == (DestinLayer& o);

	struct FamilySizes {
		int nLargeFamilies; //how many of the nodes have the larger number of children
		int nSmallFamilies; //hom many of the nodes have 1 less child
		int smallFamilySize; //how many children in the smaller family
		int largeFamilySize; //how many children in the larger family. largeFamilySize = smallFamilySize + 1
	};

	/**
	 * a family is it a parent node and its children nodes, the family size the
	 * the number of children in the family. If the input layer node count
	 * is not evenly divisible by the parent layer node count, then some
	 * nodes in the parent layer will have one more or less one less child than
	 * the others.
	 *
     * @param parentLayerNodeCount
     * @param childNodeCount
     * @return see FamilySizes struct
     */
	static FamilySizes calcFamilySizes(int parentLayerNodeCount, int childNodeCount);
};

#endif
