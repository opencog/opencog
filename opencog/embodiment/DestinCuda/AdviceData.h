#ifndef ADVICE_DATA_H
#define ADVICE_DATA_H

#include <vector>
#include <fstream>
using namespace std;

enum OutputTypes
{
	eBeliefs,
	eBeliefInAdviceTabular,
	eBeliefInAdviceNNFA,
	eBeliefInAdviceLinearFA,
};


class AdviceData
{
private:
	//The original idea for advice was the maximum belief index of the parent. This is mWinningLabel.
	int mWinningLabel;
	int mMovementBeingProcessed;  //this is for diagnostic purposes

	// However we want to extend beyond that so there are at least two things we can pass back.  One is the
	// entire set of beliefs of the parent, which we could extract the winning label from if we want.  Another is the
	// unsupervised clustering we are doing on the entire set of movements which generally will not be available until the end of the sequence.
	// or even both.
	// There may be others we will want but we can start with these. However note that for now we will not write these to disk,
	// as they are mostly for scratch.
	float* mParentBeliefs;
	bool mbValidParentUnsupervisedAdvice;
	float* mParentUnsupervisedMemberships;
	int mNumberOfParentUnsupervisedMemberships;
	int mNumberOfParentBeliefs;
public:

	// the following are used for debugging so we can track the flow of advice through the system.
	int mTempID;
	int mCountNumber; //this tells us the count we are on, basically its -1 and incremented each time movement being processed hits 0

	AdviceData(void);

	AdviceData(AdviceData& oN);

	~AdviceData(void);

	void SetID(int L,int R,int C);

	void SetParentBeliefs( float* pBeliefs, int iN );

	void SetParentUnsupervisedAdvice( float* pUnsup, int iN );

	bool GetValidParentUnsupervisedAdvice();

	AdviceData& operator = (AdviceData& oN);
	bool operator == (AdviceData& o);
	bool operator != (AdviceData& o);

	void ClearAdvice();

	int GetBestPSSATableIndex(){ return mWinningLabel;};

	int GetMovementBeingProcessed(){ return mMovementBeingProcessed; };

	void SetMovementAndBestPSSATableIndex(int Movement,int Index);
	
	//Write to a stream...
	bool WriteToStream(std::ofstream& stmOutput);

	// Read from a stream...
	bool ReadFromStream(std::ifstream& stmInput);	

	float* GetPointerToParentUnsupervisedMemberships(){ return mParentUnsupervisedMemberships; };

	int GetNumberOfParentUnsupervisedMemberships(){ return mNumberOfParentUnsupervisedMemberships; };
};

#endif
