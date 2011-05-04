#include "AdviceData.h"

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <ctime>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#include <string>
#else
// Linux only requirements...
#include <string.h>
#include <stdlib.h>
#endif

using namespace std;

AdviceData::AdviceData(void)
{
	mParentBeliefs = NULL;
	mParentUnsupervisedMemberships = NULL;
	ClearAdvice();
	mTempID = -1;
	mCountNumber = -1;
}


void AdviceData::SetID(int L,int R,int C)
{ 
	mTempID = 1000*L+100*R+C; 
};


void AdviceData::SetMovementAndBestPSSATableIndex(int Movement,int Index)
{
	mMovementBeingProcessed=Movement; 
	if ( mMovementBeingProcessed==0 )
	{
		mCountNumber++; 
	}

	mWinningLabel = Index;
}


AdviceData::~AdviceData(void)
{
}

AdviceData::AdviceData(AdviceData& oN)
{
	mParentBeliefs = NULL;
	mParentUnsupervisedMemberships = NULL;
	ClearAdvice();
	*this = oN;
}

AdviceData& AdviceData::operator = (AdviceData& oN)
{
	mTempID = oN.mTempID;
	mCountNumber = oN.mCountNumber;

	ClearAdvice();
	mWinningLabel = oN.mWinningLabel;
	mMovementBeingProcessed = oN.mMovementBeingProcessed;
	mbValidParentUnsupervisedAdvice = oN.mbValidParentUnsupervisedAdvice;

	mNumberOfParentUnsupervisedMemberships = oN.mNumberOfParentUnsupervisedMemberships;
	mNumberOfParentBeliefs = oN.mNumberOfParentBeliefs;

	if ( mNumberOfParentBeliefs != 0 )
	{
		mParentBeliefs = new float[mNumberOfParentBeliefs];
		memcpy( mParentBeliefs,oN.mParentBeliefs,sizeof(*mParentBeliefs)*mNumberOfParentBeliefs);
	}

	if ( mNumberOfParentUnsupervisedMemberships != 0 )
	{
	   mParentUnsupervisedMemberships = new float[mNumberOfParentUnsupervisedMemberships];
		memcpy( mParentUnsupervisedMemberships,oN.mParentUnsupervisedMemberships,sizeof(*mParentUnsupervisedMemberships)*mNumberOfParentUnsupervisedMemberships);
	}
	return *this;
}

bool AdviceData::operator == (AdviceData& o)
{
	if ( mWinningLabel!=o.mWinningLabel ) return false;
	if ( mMovementBeingProcessed != o.mMovementBeingProcessed ) return false;
	return true;
}

bool AdviceData::operator != (AdviceData& o)
{
	return !(*this==o);
}

void AdviceData::ClearAdvice()
{
	mbValidParentUnsupervisedAdvice = false;
	mWinningLabel=0;
	mMovementBeingProcessed = -1;
	mNumberOfParentUnsupervisedMemberships=0;
	mNumberOfParentBeliefs=0;
	if ( mParentBeliefs != NULL )
	{
		delete mParentBeliefs;
		mParentBeliefs=NULL;
	}
	if ( mParentUnsupervisedMemberships != NULL )
	{
		delete mParentUnsupervisedMemberships;
		mParentUnsupervisedMemberships=NULL;
	}

}

bool AdviceData::GetValidParentUnsupervisedAdvice()
{
	return mbValidParentUnsupervisedAdvice;
}

bool AdviceData::WriteToStream(std::ofstream& stmOutput)
{
	int iFlag = -2020;
	stmOutput.write( (char*)&iFlag, sizeof(iFlag) );
	stmOutput.write( (char*)&mWinningLabel, sizeof(mWinningLabel) );
	stmOutput.write( (char*)&mMovementBeingProcessed, sizeof(mMovementBeingProcessed) );
	return true;
}

void AdviceData::SetParentBeliefs( float* pBeliefs, int iN )
{
	if ( mParentBeliefs==NULL )
	{
		mParentBeliefs = new float[iN];
	}
	memcpy( mParentBeliefs, pBeliefs, iN*sizeof(*pBeliefs) );
	mNumberOfParentBeliefs=iN;
}

void AdviceData::SetParentUnsupervisedAdvice( float* pUnsup, int iN )
{
	if ( pUnsup==NULL )
	{
		mbValidParentUnsupervisedAdvice = false;
	}
	else
	{
		if ( mParentUnsupervisedMemberships==NULL )
		{
			mParentUnsupervisedMemberships = new float[iN];
		}
		memcpy( mParentUnsupervisedMemberships, pUnsup, iN*sizeof(*pUnsup) );
		mbValidParentUnsupervisedAdvice = true;
		mNumberOfParentUnsupervisedMemberships = iN;
	}
}

bool AdviceData::ReadFromStream(std::ifstream& stmInput)
{
	if ( mParentBeliefs != NULL )
	{
		delete mParentBeliefs;
		mParentBeliefs=NULL;
	}
	if ( mParentUnsupervisedMemberships != NULL )
	{
		mParentUnsupervisedMemberships=NULL;
		delete mParentUnsupervisedMemberships;
	}
	ClearAdvice();
	int iFlag;
	stmInput.read( (char*)&iFlag, sizeof(iFlag) );
	if ( iFlag==-2020 )
	{
		stmInput.read( (char*)&mWinningLabel, sizeof(mWinningLabel) );
		stmInput.read( (char*)&mMovementBeingProcessed, sizeof(mMovementBeingProcessed) );
	}
	else
	{
		throw "Exception reading AdviceData from stream!";
	}
	return true;
}
