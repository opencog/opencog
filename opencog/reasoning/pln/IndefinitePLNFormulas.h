#ifndef _INDEFINITEPLNFORMULAS_H
#define _INDEFINITEPLNFORMULAS_H

//#include "Formula.h"


#include <math.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_integration.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#include <IndefiniteTruthValue.h>

#include "deductionLookupTable.h"
#include "Formula.h"
     
#include <assert.h>
#include <algorithm>
#include <vector>

using namespace opencog;

typedef union _tscreg_t {
    struct _l {
        unsigned int high;
        unsigned int low;
    } l;
    uint64_t ll;
} tscreg_t;

/* TSC access code */
static inline void get_tsc(tscreg_t *t)
{
    __asm__ __volatile__ (
        "rdtsc"
        : "=a" (t->l.high), "=d" (t->l.low));
}

/// Formula defined in the integral of step one [(x-L1)^ks * (U1-x)^k(1-s)
double integralFormula (double x, void * params);

/**
 * Compute the integral of (x-L1)^ks * (U1-x)^k(1-s) between lowerBound and upperBound, 
 * which is in numerator and denominator of the step 1 formula (compute diff) 
 */
float computeDensityFunctionIntegral(float lowerBound,
      								float upperBound,
									float L_,
									float U_,
									float k_,
									float s_);
/**
 * This method is responsiple of computing the diff, which is used in computing 
 * L1 and U1 of step 1. L1 = L-diff and U1=U+diff
 */
float computeDiff(IndefiniteTruthValue* TV);

float generateRandomValueBetaDistribution(double a, double b);

/**
 * Use GSL to generate n random values of beta distribution
 * @param values the result
 * @param n the number of random values
 * @param a alfa parameter of beta distribution
 * @param b beta parameter of beta distribution
 * 
 */
void generateRandomValueBetaDistribution(float values[], int n, double a, double b);

float scaleRandomValue(float value, float L_, float U_);

/**
 * This method is related to step 2.2. All values are scaled according to
 * the formula: values[i]=L_+(U_-L_)*values[i]. Remembering that values[i]
 * must be between 0.0 and 1.0. Otherwise, it is truncated.
 */
void scaleRandomValue(float values[], int size, float L_, float U_);

void generateFirstOrderDistributions(IndefiniteTruthValue* TV);

/**
 * This is the last step of the procedure. After the rule computation, the
 * conclusion has to be found.
 * For each first order distribution (n1), it is found the mean value averaging the n2 values of the
 * distribution. The average mean (s) is computed averaging over all n1 elements of the means list.
 * The 
 * 
 */
IndefiniteTruthValue* findConclusion(vector<float> means, float b=IndefiniteTruthValue::DEFAULT_CONFIDENCE_LEVEL);

vector<float> computeMeans(vector<vector<float> > distribution);
///old version
vector<float> computeMeans(vector<float*> distribution);

/*
 * truncate the value to 0.0 if it is less than 0.0 or to 1.0 if it is greater than 1.0
 */
void truncate(float &value);

namespace reasoning {

static float s=0.5f;
static float diffError=0.001f;
static int n1=100;
static int n2=100;
const float IndefiniteMembershipToExtensionalInheritanceCountDiscountFactor = 1.5f;
static bool SAVE_DEDUCTION_LOOKUP_TABLE=true;
static bool USE_DEDUCTION_LOOKUP_TABLE=false;

static void setSaveDeductionLookupTable(bool b){SAVE_DEDUCTION_LOOKUP_TABLE=b;}
static void setUseDeductionLookupTable(bool b){USE_DEDUCTION_LOOKUP_TABLE=b;}

//number of times before identifying a set of premieses as inconsistent
//the inconsistency may occur because the premises are not valid
const int MAX_CONSISTENCY_COUNT=100;
const float INCONSISTENCY_VALUE=-100.0f;

class IndefiniteSymmetricANDFormula : public Formula<2>
{
public:
	TruthValue* simpleCompute(TruthValue** TV, int N, long U = DefaultU) const
	{
    assert(N == 2);
		assert(TV[0]); assert(TV[1]);
		IndefiniteTruthValue* TVA = (IndefiniteTruthValue*)(TV[0]);
		IndefiniteTruthValue* TVB = (IndefiniteTruthValue*)(TV[1]);
		IndefiniteTruthValue* result;
		
		float diffA = computeDiff(TVA);
//		printf("Diff-A: %.3f\n\n",diffA); 
		float diffB = computeDiff(TVB);
//		printf("DiffB: %.3f\n\n",diffB);
		
		generateFirstOrderDistributions(TVA);
		generateFirstOrderDistributions(TVB);
		
		vector<float*> distributionA = TVA->getFirstOrderDistribution();
		vector<float*> distributionB = TVB->getFirstOrderDistribution();
		vector<vector<float> > distributionResult (n1, vector<float>(n2));
						
		for(int i=0; i<n1; i++)
		{
			for(int j=0; j<n2; j++)
			{
				distributionResult[i][j]=distributionA[i][j]*distributionB[i][j];
			}
		}
		
		vector<float> means=computeMeans(distributionResult);
		result = findConclusion(means);
		return result;
	}
};

//modus pones
class IndefiniteSymmetricImplicationBreakdownFormula : public Formula<2>
{
public:
	TruthValue* simpleCompute(TruthValue** TV, int N, long U = DefaultU) const
	{
        assert(N == 2);
        assert(TV[0]); assert(TV[1]);
		IndefiniteTruthValue* linkAB = (IndefiniteTruthValue*)(TV[0]);
		IndefiniteTruthValue* TVA = (IndefiniteTruthValue*)(TV[1]);
		
		float diffA=computeDiff(TVA);
//		printf("Diff-A: %.3f\n\n",diffA); 
		float diffAB=computeDiff(linkAB);
//		printf("Diff=AB: %.3f\n\n",diffAB);
		
		generateFirstOrderDistributions(TVA);
		generateFirstOrderDistributions(linkAB);
			
		vector<float*> distributionA = TVA->getFirstOrderDistribution();
		vector<float*> distributionAB = linkAB->getFirstOrderDistribution();
		vector<vector<float> > Q (n1, vector<float>(n2));		
		vector<vector<float> > R (n1, vector<float>(n2));
		
		for(int i=0; i<n1; i++)
		{
			for(int j=0; j<n2; j++)
			{
				Q[i][j]=distributionA[i][j]*distributionAB[i][j];
				//R[i][j]=1-distributionA[i][j]+distributionA[i][j]*distributionAB[i][j];
                R[i][j]=1-distributionA[i][j]+Q[i][j];//Q[i][j] is distributionA[i][j]*distributionAB[i][j]
			}
		}
		return conclusion(Q,R);
	}
	
	IndefiniteTruthValue* conclusion(vector<vector<float> >& Q, vector<vector<float> >& R, float b=IndefiniteTruthValue::DEFAULT_CONFIDENCE_LEVEL) const
//	IndefiniteTruthValue* conclusion(vector<float*>& Q, vector<float*>& R, float b=IndefiniteTruthValue::DEFAULT_CONFIDENCE_LEVEL) const
	{
		vector<float> meansQ, meansR, meansQR;
		float meanQ, meanR;
		float lower, upper;
		double middle_d=floor(b*n1);
		int middle=int(middle_d);
		float rate=0.001f;
		
		for(int i=0; i<n1; i++)
		{
			meanQ=0.0; meanR=0.0;
			for(int j=0; j<n2; j++)
			{
				meanQ += Q[i][j];
				meanR += R[i][j];
			}
			meansQ.push_back((float)(meanQ/n2));
			meansR.push_back((float)(meanR/n2));
			printf("meansQ = %03f meansR = %03f \n", meanQ/n2, meanR/n2);
		}

		lower=meansQ[0];
		upper=meansQ[0];
		for(int i=0; i<n1; i++)
		{
			meansQR.push_back((meansQ[i]+meansR[i])/2);
			
			if(meansQ[i] < lower)
			{
				lower = meansQ[i];
			}
			if(meansR[i] > upper)
			{
				upper=meansR[i];
			}
			printf("lower = %03f upper = %03f\n", lower, upper);
		}
		
		float strength=0.0;

		int count;
		while (true)
		{
			count=0;
			lower=lower+rate;
			upper=upper-rate;

			for(int i=0;i<n1;i++)
			{
				strength += meansQR[i];
				if((float)meansQ[i] >= lower && (float)meansR[i] <= upper)
				{
					count++;
				}
			}
			if(count <= middle)
			{
				if(count < middle)
				{
					lower=(lower+(lower-rate))/2;
					upper=(upper+(upper+rate))/2;
				}
				IndefiniteTruthValue* result= new IndefiniteTruthValue(lower,upper);
				result->setMean(strength/n1);
				return result;
			}
		}
	}
};


class IndefiniteSymmetricRevisionFormula : public Formula<2>
{
public:
	TruthValue* simpleCompute(TruthValue** TV, int N, long U = DefaultU) const
	{
        assert(N == 2);
		assert(TV[0]); assert(TV[1]);
		IndefiniteTruthValue* TVA = (IndefiniteTruthValue*)(TV[0]);
		IndefiniteTruthValue* TVB = (IndefiniteTruthValue*)(TV[1]);
		IndefiniteTruthValue* result;
		
		float diffA=computeDiff(TVA);
//		printf("Diff-A: %.3f\n\n",diffA);
		float diffB=computeDiff(TVB);
//		printf("Diff=B: %.3f\n\n",diffB);
		
		generateFirstOrderDistributions(TVA);
		generateFirstOrderDistributions(TVB);
		
		vector<float*> distributionA = TVA->getFirstOrderDistribution();
		vector<float*> distributionB = TVB->getFirstOrderDistribution();
		
		vector<vector<float> > distributionResult (n1, vector<float>(n2));

		float W1=TVA->getU()-TVA->getL();
		float W2=TVB->getU()-TVB->getL();
		
		float n1_=IndefiniteTruthValue::DEFAULT_K*(1-W1)/W1;
		float n2_=IndefiniteTruthValue::DEFAULT_K*(1-W2)/W2;
		
		float w1=n1_/(n1_+n2_);
		float w2=n2_/(n1_+n2_);
		
		for(int i=0; i<n1; i++)
		{
			for(int j=0; j<n2; j++)
			{
				distributionResult[i][j]=(w1*distributionA[i][j])+(w2*distributionB[i][j]);
			}
		}
		
		vector<float> means=computeMeans(distributionResult);
		result = findConclusion(means);
		return result;
	}
};

class IndefiniteSymmetricAbductionFormula : public Formula<5>
{
public:
	TruthValue* simpleCompute(TruthValue** TV, int N, long U = DefaultU) const
	{
        assert(N == 5);
		assert(TV[0]); assert(TV[1]);assert(TV[2]); assert(TV[3]);assert(TV[4]);
		IndefiniteTruthValue* TVA = (IndefiniteTruthValue*)(TV[0]);
		IndefiniteTruthValue* TVB = (IndefiniteTruthValue*)(TV[1]);
		IndefiniteTruthValue* TVC = (IndefiniteTruthValue*)(TV[2]);
		IndefiniteTruthValue* TVAB = (IndefiniteTruthValue*)(TV[3]);
		IndefiniteTruthValue* TVCB = (IndefiniteTruthValue*)(TV[4]);
		IndefiniteTruthValue* result;
		
		float diffA=computeDiff(TVA);
//		printf("Diff-A: %.3f\n\n",diffA);
		float diffB=computeDiff(TVB);
//		printf("Diff=B: %.3f\n\n",diffB);
		float diffC=computeDiff(TVC);
//		printf("Diff=C: %.3f\n\n",diffC);
		float diffAB=computeDiff(TVAB);
//		printf("Diff-AB: %.3f\n\n",diffAB);
		float diffCB=computeDiff(TVCB);
//		printf("Diff=BC: %.3f\n\n",diffCB);

		vector<vector<float> > distributionA (n1, vector<float>(n2));
		vector<vector<float> > distributionB(n1, vector<float>(n2));
		vector<vector<float> > distributionC(n1, vector<float>(n2));
		vector<vector<float> > distributionAB(n1, vector<float>(n2));
		vector<vector<float> > distributionCB(n1, vector<float>(n2));
		vector<vector<float> > distributionAC(n1, vector<float>(n2));
		
		vector<float> valuesA = vector<float>(n1,0.0f);
		vector<float> valuesB = vector<float>(n1,0.0f);
		vector<float> valuesC = vector<float>(n1,0.0f);
		vector<float> valuesAB = vector<float>(n1,0.0f);
		vector<float> valuesCB = vector<float>(n1,0.0f);
		
		float alpha=IndefiniteTruthValue::DEFAULT_K*0.5;
		float beta=alpha;
		//generate n1 consistency values for each TV
		for(int i=0; i<n1; i++)
		{
			do
			{
				valuesA[i]=scaleRandomValue(
						generateRandomValueBetaDistribution(alpha,beta),TVA->getL_(), TVA->getU_());
				valuesB[i]=scaleRandomValue(
						generateRandomValueBetaDistribution(alpha,beta),TVB->getL_(), TVB->getU_());
				valuesC[i]=scaleRandomValue(
						generateRandomValueBetaDistribution(alpha,beta),TVC->getL_(), TVC->getU_());
				valuesAB[i]=scaleRandomValue(
						generateRandomValueBetaDistribution(alpha,beta),TVAB->getL_(), TVAB->getU_());
				valuesCB[i]=scaleRandomValue(
						generateRandomValueBetaDistribution(alpha,beta),TVCB->getL_(), TVCB->getU_());
			}while(
				valuesAB[i] < GSL_MAX((valuesA[i] + valuesB[i] - 1)/valuesA[i],0) ||
				valuesAB[i] > GSL_MIN(valuesB[i]/valuesA[i],1) ||
				valuesCB[i] < GSL_MAX((valuesC[i] + valuesB[i] - 1)/valuesC[i],0) ||
				valuesCB[i] > GSL_MIN(valuesB[i]/valuesC[i],1)
			);	
		}

		for(int i=0;i<n1;i++)
		{
			truncate(valuesA[i]);
			truncate(valuesB[i]);
			truncate(valuesC[i]);
			truncate(valuesAB[i]);
			truncate(valuesCB[i]);
		}
		for(int i=0; i<n1; i++)
		{
			for(int j=0; j<n2; j++)
			{
				do
				{
					distributionA[i][j]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*valuesA[i], IndefiniteTruthValue::DEFAULT_K*(1-valuesA[i]));
					distributionB[i][j]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*valuesB[i], IndefiniteTruthValue::DEFAULT_K*(1-valuesB[i]));
					distributionC[i][j]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*valuesC[i], IndefiniteTruthValue::DEFAULT_K*(1-valuesC[i]));
					distributionAB[i][j]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*valuesAB[i], IndefiniteTruthValue::DEFAULT_K*(1-valuesAB[i]));
					distributionCB[i][j]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*valuesCB[i], IndefiniteTruthValue::DEFAULT_K*(1-valuesCB[i]));
				}while(
					distributionAB[i][j] < GSL_MAX((distributionA[i][j] + distributionB[i][j] - 1)/distributionA[i][j],0) ||
					distributionAB[i][j] > GSL_MIN(distributionB[i][j]/distributionA[i][j],1) ||
					distributionCB[i][j] < GSL_MAX((distributionC[i][j] + distributionB[i][j] - 1)/distributionC[i][j],0) ||
					distributionCB[i][j] > GSL_MIN(distributionB[i][j]/distributionC[i][j],1)
				);
			}
		}
		
		//sAC = sAB sCB sC / sB + (1-sAB) (1– sCB ) sC / (1- sB ).
		for(int i=0; i<n1; i++)
		{
			for(int j=0; j<n2; j++)
			{
				truncate(distributionA[i][j]);
				truncate(distributionB[i][j]);
				truncate(distributionC[i][j]);
				truncate(distributionAB[i][j]);
				truncate(distributionCB[i][j]);

				if(distributionB[i][j]<=0.000001){
					distributionAC[i][j]=distributionC[i][j];
				}else{
					if(distributionB[i][j]>=0.999999){
						distributionAC[i][j]=1.0;
					}else{
						distributionAC[i][j]=distributionAB[i][j]*distributionCB[i][j]*distributionC[i][j]/distributionB[i][j]+
							(1-distributionAB[i][j])*(1-distributionCB[i][j])*distributionC[i][j]/(1-distributionB[i][j]);						
					}
				}
			}
		}
		
		vector<float> means=computeMeans(distributionAC);
		result = findConclusion(means);
		return result;
	}
};

class IndefiniteMem2InhFormula : public Formula<1>
{
public:
	TruthValue* simpleCompute(TruthValue** TV, int N, long U = DefaultU) const
	{
		assert(N==1);
		assert(TV[0]);
		IndefiniteTruthValue* TVA = (IndefiniteTruthValue*)(TV[0]);
		float width=TVA->getU() - TVA->getL();
		float center=TVA->getL()+width/2;
		float L_ = center - ((width*IndefiniteMembershipToExtensionalInheritanceCountDiscountFactor)/2);
		float U_ = center + ((width*IndefiniteMembershipToExtensionalInheritanceCountDiscountFactor)/2);
		truncate(L_);
		truncate(U_);
		IndefiniteTruthValue* result= new IndefiniteTruthValue(L_,U_);
		result->setMean(TVA->getMean());
		return result;			
	}
};

class IndefiniteInh2MemFormula : public Formula<1>
{
public:
	TruthValue* simpleCompute(TruthValue** TV, int N, long U = DefaultU) const
	{
		assert(N==1);
		assert(TV[0]);
		IndefiniteTruthValue* TVA = (IndefiniteTruthValue*)(TV[0]);
		float width=TVA->getU() - TVA->getL();
		float center=TVA->getL()+width/2;
//		float L_ = center - ((width*IndefiniteMembershipToExtensionalInheritanceCountDiscountFactor)/2);
//		float U_ = center + ((width*IndefiniteMembershipToExtensionalInheritanceCountDiscountFactor)/2);
		float L_ = center - ((width*1.2)/2);
		float U_ = center + ((width*1.2)/2);		
		truncate(L_);
		truncate(U_);
		IndefiniteTruthValue* result=new IndefiniteTruthValue(L_,U_);
		result->setMean(TVA->getMean());
		return result;			
	}
};


class IndefiniteSymmetricBayesFormula : public Formula<3>
{
public:
	TruthValue* simpleCompute(TruthValue** TV, int N, long U = DefaultU) const 
	{
    assert(N == 3);
		assert(TV[0]); assert(TV[1]);assert(TV[2]);
		IndefiniteTruthValue* TVA = (IndefiniteTruthValue*)(TV[0]);
		IndefiniteTruthValue* TVC = (IndefiniteTruthValue*)(TV[1]);
		IndefiniteTruthValue* TVAC = (IndefiniteTruthValue*)(TV[2]);
		IndefiniteTruthValue* result;
		
		float diffA=computeDiff(TVA);
		printf("Diff-A: %.3f\n\n",diffA);
		float diffC=computeDiff(TVC);
		printf("Diff=C: %.3f\n\n",diffC);
		float diffAC=computeDiff(TVAC);
		printf("Diff-AC: %.3f\n\n",diffAC);

		vector<vector<float> > distributionA (n1, vector<float>(n2));
		vector<vector<float> > distributionC(n1, vector<float>(n2));
		vector<vector<float> > distributionAC(n1, vector<float>(n2));
		vector<vector<float> > distributionCA(n1, vector<float>(n2));

		vector<float> valuesA = vector<float>(n1,0.0f);
		vector<float> valuesC = vector<float>(n1,0.0f);
		vector<float> valuesAC = vector<float>(n1,0.0f);
		
		float alpha=IndefiniteTruthValue::DEFAULT_K*0.5;
		float beta=alpha;
		
		//generate n1 consistency values for each TV
		for(int i=0; i<n1; i++)
		{
			do
			{
				valuesA[i]=scaleRandomValue(
						generateRandomValueBetaDistribution(alpha,beta),TVA->getL_(), TVA->getU_());
				valuesC[i]=scaleRandomValue(
						generateRandomValueBetaDistribution(alpha,beta),TVC->getL_(), TVC->getU_());
				valuesAC[i]=scaleRandomValue(
						generateRandomValueBetaDistribution(alpha,beta),TVAC->getL_(), TVAC->getU_());
			}while(
				valuesAC[i] < GSL_MAX((valuesA[i] + valuesC[i] - 1)/valuesA[i],0) ||
				valuesAC[i] > GSL_MIN(valuesC[i]/valuesA[i],1)
			);
		}
		
		for(int i=0; i<n1; i++)
		{
			truncate(valuesA[i]);
			truncate(valuesC[i]);
			truncate(valuesAC[i]);
		}
		
		for(int i=0; i<n1; i++)
		{
			for(int j=0; j<n2; j++)
			{
				do
				{
					distributionA[i][j]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*valuesA[i], IndefiniteTruthValue::DEFAULT_K*(1-valuesA[i]));
					distributionC[i][j]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*valuesC[i], IndefiniteTruthValue::DEFAULT_K*(1-valuesC[i]));
					distributionAC[i][j]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*valuesAC[i], IndefiniteTruthValue::DEFAULT_K*(1-valuesAC[i]));
				}while(
					distributionAC[i][j] < GSL_MAX((distributionA[i][j] + distributionC[i][j] - 1)/distributionA[i][j],0) ||
					distributionAC[i][j] > GSL_MIN(distributionC[i][j]/distributionA[i][j],1)
				);
			}
		}
		
		for(int i=0; i<n1; i++)
		{
			for(int j=0; j<n2; j++)
			{
				truncate(distributionA[i][j]);
				truncate(distributionC[i][j]);
				truncate(distributionAC[i][j]);
				
				if(distributionC[i][j]<=0.000001){
					distributionCA[i][j]=0.0;
				}else{
					distributionCA[i][j]=(distributionA[i][j]*distributionAC[i][j])/distributionC[i][j];
				}
			}
		}
		vector<float> means=computeMeans(distributionCA);
		result = findConclusion(means);
		return result;
	}
};


class IndefiniteSymmetricDeductionFormula : public Formula<5>
{
public:
	TruthValue* simpleCompute(TruthValue** TV, int N, long U = DefaultU) const
	{
    assert(N == 5);
		assert(TV[0]); assert(TV[1]); assert(TV[2]); assert(TV[3]);assert(TV[4]);
		IndefiniteTruthValue* TVA = (IndefiniteTruthValue*)(TV[0]);
		IndefiniteTruthValue* TVB = (IndefiniteTruthValue*)(TV[1]);
		IndefiniteTruthValue* TVC = (IndefiniteTruthValue*)(TV[2]);
		IndefiniteTruthValue* TVAB = (IndefiniteTruthValue*)(TV[3]);
		IndefiniteTruthValue* TVBC = (IndefiniteTruthValue*)(TV[4]);
		IndefiniteTruthValue* result;
		
		vector<float> means;
		
		float diffA=computeDiff(TVA);
//		printf("Diff-A: %.3f\n\n",diffA);
		float diffB=computeDiff(TVB);
//		printf("Diff=B: %.3f\n\n",diffB);
		float diffC=computeDiff(TVC);
//		printf("Diff=C: %.3f\n\n",diffC);
		float diffAB=computeDiff(TVAB);
//		printf("Diff-AB: %.3f\n\n",diffAB);
		float diffBC=computeDiff(TVBC);
//		printf("Diff=BC: %.3f\n\n",diffBC);

		vector<vector<float> > distributionA (n1, vector<float>(n2));
		vector<vector<float> > distributionB(n1, vector<float>(n2));
		vector<vector<float> > distributionC(n1, vector<float>(n2));
		vector<vector<float> > distributionAB(n1, vector<float>(n2));
		vector<vector<float> > distributionBC(n1, vector<float>(n2));
		vector<vector<float> > distributionAC(n1, vector<float>(n2));
		
		vector<float> valuesA = vector<float>(n1,0.0f);
		vector<float> valuesB = vector<float>(n1,0.0f);
		vector<float> valuesC = vector<float>(n1,0.0f);
		vector<float> valuesAB = vector<float>(n1,0.0f);
		vector<float> valuesBC = vector<float>(n1,0.0f);
		
		float alpha=IndefiniteTruthValue::DEFAULT_K*0.5;
		float beta=alpha;	
		int consistencyCount=0;
		//generate n1 consistency values for each TV
		for(int i=0; i<n1; i++)
		{
			do
			{
				consistencyCount++;
				valuesA[i]=scaleRandomValue(
						generateRandomValueBetaDistribution(alpha,beta),TVA->getL_(), TVA->getU_());
				valuesB[i]=scaleRandomValue(
						generateRandomValueBetaDistribution(alpha,beta),TVB->getL_(), TVB->getU_());
				valuesC[i]=scaleRandomValue(
						generateRandomValueBetaDistribution(alpha,beta),TVC->getL_(), TVC->getU_());
				valuesAB[i]=scaleRandomValue(
						generateRandomValueBetaDistribution(alpha,beta),TVAB->getL_(), TVAB->getU_());
				valuesBC[i]=scaleRandomValue(
						generateRandomValueBetaDistribution(alpha,beta),TVBC->getL_(), TVBC->getU_());
						
				//the set of premises is inconsistent
				if(consistencyCount >= MAX_CONSISTENCY_COUNT)
				{
					result=new IndefiniteTruthValue(0.0f, 1.0f, 1.0f);
					distributionA.clear();
					distributionB.clear();
					distributionC.clear();
					distributionAB.clear();
					distributionBC.clear();
					distributionAC.clear();
					return result;
				}						
				
			}while(
				valuesAB[i] < GSL_MAX((valuesA[i] + valuesB[i] - 1)/valuesA[i],0) ||
				valuesAB[i] > GSL_MIN(valuesB[i]/valuesA[i],1) ||
				valuesBC[i] < GSL_MAX((valuesB[i] + valuesC[i] - 1)/valuesB[i],0) ||
				valuesBC[i] > GSL_MIN(valuesC[i]/valuesB[i],1)
			);
			
			truncate(valuesA[i]);
			truncate(valuesB[i]);
			truncate(valuesC[i]);
			truncate(valuesAB[i]);
			truncate(valuesBC[i]);
			

			if(USE_DEDUCTION_LOOKUP_TABLE)
			{
				float mean=DeductionLookupTable::getInstance()->lookup(valuesA[i], valuesB[i], valuesC[i], valuesAB[i], valuesBC[i]);
				printf("<A, B,C,AB,BC>=AC -- <%.3f,%.3f,%.3f,%.3f,%.3f>=%.3f\n",valuesA[i],valuesB[i], valuesC[i], valuesAB[i], valuesBC[i],mean);
				
                //check if it is a consistent value
				if(mean>0.0){
					means.push_back(mean);
				}
			}
		}

		if(USE_DEDUCTION_LOOKUP_TABLE)
		{
			result=findConclusion(means);
			distributionA.clear();
			distributionB.clear();
			distributionC.clear();
			distributionAB.clear();
			distributionBC.clear();
			distributionAC.clear();
			return result;
		}

		for(int i=0; i<n1; i++)
		{
			for(int j=0; j<n2; j++)
			{
				do
				{
					distributionA[i][j]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*valuesA[i], IndefiniteTruthValue::DEFAULT_K*(1-valuesA[i]));
					distributionB[i][j]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*valuesB[i], IndefiniteTruthValue::DEFAULT_K*(1-valuesB[i]));
					distributionC[i][j]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*valuesC[i], IndefiniteTruthValue::DEFAULT_K*(1-valuesC[i]));
					distributionAB[i][j]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*valuesAB[i], IndefiniteTruthValue::DEFAULT_K*(1-valuesAB[i]));
					distributionBC[i][j]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*valuesBC[i], IndefiniteTruthValue::DEFAULT_K*(1-valuesBC[i]));
				}while(
					distributionAB[i][j] < GSL_MAX((distributionA[i][j] + distributionB[i][j] - 1)/distributionA[i][j],0) ||
					distributionAB[i][j] > GSL_MIN(distributionB[i][j]/distributionA[i][j],1) ||
					distributionBC[i][j] < GSL_MAX((distributionB[i][j] + distributionC[i][j] - 1)/distributionB[i][j],0) ||
					distributionBC[i][j] > GSL_MIN(distributionC[i][j]/distributionB[i][j],1)
				);
			}
		}

		for(int i=0; i<n1; i++)
		{
			for(int j=0; j<n2; j++)
			{
				
				truncate(distributionA[i][j]);
				truncate(distributionB[i][j]);
				truncate(distributionC[i][j]);
				truncate(distributionAB[i][j]);
				truncate(distributionBC[i][j]);
				
				if(distributionB[i][j] >= 0.999999){
					distributionAC[i][j]=distributionAB[i][j]*distributionBC[i][j];
				}else{
					distributionAC[i][j]=
						distributionAB[i][j] * distributionBC[i][j] + (1 - distributionAB[i][j]) *
							(distributionC[i][j] - distributionB[i][j] * distributionBC[i][j])/(1 - distributionB[i][j]);
				}
			}
		}
		means=computeMeans(distributionAC);
		
		//Salva no lookuptable
/*		if(SAVE_DEDUCTION_LOOKUP_TABLE)
		{
			for(int i=0; i<n1; i++){
																			 		 
				DeductionLookupTable::getInstance()->add(valuesB[i],
														 valuesC[i],
														 valuesAB[i],
														 valuesBC[i],
														 means[i]);
																					 		 
			}
		}	
 */   
		result=findConclusion(means);
		distributionA.clear();
		distributionB.clear();
		distributionC.clear();
		distributionAB.clear();
		distributionBC.clear();
		distributionAC.clear();
		return result;
	}
};

class BuildLookuptTableIndefiniteSymmetricDeductionFormula
{
public:
	void compute(long U = DefaultU) const
	{
		
		vector<float> means;
		
/*		vector<vector<float> > distributionA (n1, vector<float>(n2));
		vector<vector<float> > distributionB(n1, vector<float>(n2));
		vector<vector<float> > distributionC(n1, vector<float>(n2));
		vector<vector<float> > distributionAB(n1, vector<float>(n2));
		vector<vector<float> > distributionBC(n1, vector<float>(n2));
		vector<vector<float> > distributionAC(n1, vector<float>(n2));
		
		float valuesA[n1];
		float valuesB[n1];
		float valuesC[n1];
		float valuesAB[n1];
		float valuesBC[n1];
*/

		vector<float> distributionA = vector<float>(n2,0.0f);
		vector<float> distributionB = vector<float>(n2,0.0f);
		vector<float> distributionC = vector<float>(n2,0.0f);
		vector<float> distributionAB = vector<float>(n2,0.0f);
		vector<float> distributionBC = vector<float>(n2,0.0f);
		vector<float> distributionAC = vector<float>(n2,0.0f);
        
		//float distributionA,distributionB,distributionC,distributionAB,distributionBC,distributionAC;		
		float alpha=IndefiniteTruthValue::DEFAULT_K*0.5;
		float beta=alpha;
		float A,B,C,AB,BC;
		
		//generate n1 consistency values for each TV
		float meansValues[]={0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f};
		//size=100, 200, 150, 400
		//initial=0, 100, 0, 300
		//base=0.01, 0.005, 0.0066, 0025
		
		int size=10;
		int initial=0;
		float base_value=0.1f;

        //count is used to identify when a consistency condition is not satisfied
		int count=0;		
		
		for(int iA=initial; iA<size; iA++)
		{
            A=(float)(iA*base_value);
			//distributionA=meansValues[iA];
			for(int iB=initial; iB<size; iB++)
			{
				//distributionB=meansValues[iB];
				B=(float)(iB*base_value);
				//if(B == 1.0f) break;
				for(int iC=initial; iC<size; iC++)
				{
					//distributionC=meansValues[iC];
					C=(float)(iC*base_value);
					for(int iAB=initial; iAB<size; iAB++)
					{
						//distributionAB=meansValues[iAB];
						AB=(float)(iAB*base_value);
						for(int iBC=initial; iBC<size; iBC++)
						{
							//distributionBC=meansValues[iBC];
							BC=(float)(iBC*base_value);
							//printf("<A, B, C, AB, BC>=<%.3f, %.3f, %.3f, %.3f, %.3f>\n",A,B,C,AB,BC);
                            //printf("iBC: %d\n",iBC);
                            count=0;
							for(int i=0; i<n2; i++){
                                if(count >= MAX_CONSISTENCY_COUNT){
                                  break;
                                }
                                count=0;
                                do
	    						{
                                    //if have tried more than 100 times and did not find consistency values, consider the premises as inconsistent
								    distributionA[i]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*A, IndefiniteTruthValue::DEFAULT_K*(1-A));
								    distributionB[i]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*B, IndefiniteTruthValue::DEFAULT_K*(1-B));
								    distributionC[i]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*C, IndefiniteTruthValue::DEFAULT_K*(1-C));
								    distributionAB[i]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*AB, IndefiniteTruthValue::DEFAULT_K*(1-AB));
								    distributionBC[i]=generateRandomValueBetaDistribution(IndefiniteTruthValue::DEFAULT_K*BC, IndefiniteTruthValue::DEFAULT_K*(1-BC));
    								count++;
                                    //printf("Count: %d\n",count);

	    						}while(
		    						(distributionAB[i] < GSL_MAX((distributionA[i] + distributionB[i] - 1)/distributionA[i],0) ||
			    					distributionAB[i] > GSL_MIN(distributionB[i]/distributionA[i],1) ||
				    				distributionBC[i] < GSL_MAX((distributionB[i] + distributionC[i] - 1)/distributionB[i],0) ||
					    			distributionBC[i] > GSL_MIN(distributionC[i]/distributionB[i],1)) &&
                                    (count < MAX_CONSISTENCY_COUNT)
						    	);
                            }
					
                            if(count >= MAX_CONSISTENCY_COUNT){
			    			  DeductionLookupTable::getInstance()->add(A,
				    		       							 		   B,
															 		   C,
								    						 		   AB,
															 		   BC,
															 		   INCONSISTENCY_VALUE);
                              continue;
                            }
                            
                            for(int i=0; i<n2; i++){
    							truncate(distributionA[i]);
	    						truncate(distributionB[i]);
		    					truncate(distributionC[i]);
			    				truncate(distributionAB[i]);
				    			truncate(distributionBC[i]);
							
					    		if(distributionB[i] >= 0.999999){
								    distributionAC[i]=distributionAB[i]*distributionBC[i];
    							}else{
	    							distributionAC[i]=
		    							distributionAB[i] * distributionBC[i] + (1 - distributionAB[i]) *
			    							(distributionC[i] - distributionB[i] * distributionBC[i])/(1 - distributionB[i]);
				    			}

                             }

                                float ACmean=0.0;
                                for(int i=0; i<n2; i++){
                                    ACmean += distributionAC[i];
                                }
                                ACmean=(float)((float)ACmean/(float)n2);
                                
					    		//Salva no lookuptable
						    	if(SAVE_DEDUCTION_LOOKUP_TABLE)
							    {
								    	if(ACmean >= 0.0){
									    	DeductionLookupTable::getInstance()->add(A,
                                                                                     B,
																			         C,
																			         AB,
																				     BC,
																				     ACmean);
									}
							}	
						}
					}
				}
			}
		}
		
		
		/*distributionA.clear();
		distributionB.clear();
		distributionC.clear();
		distributionAB.clear();
		distributionBC.clear();
		distributionAC.clear();
		*/
        
		DeductionLookupTable::getInstance()->closeFile();
	}
};

}//namespace
#endif /*INDEFINITEPLNFORMULAS_H */
