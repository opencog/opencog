/**
 * IndefiniteFormulaUTest.cxxtest
 *
 * Author: Fabricio - 2007-01-08
 */

#include "FormulasIndefinite.h"
#include "DeductionLookupTable.h"
#include <math.h>
#include <vector>
#include <ctime>

#define CALIBRATION_INTERVAL 1

float bvalues[6]={0.5, 0.6, 0.7, 0.8, 0.9, 0.95};
float kvalues[9]={1.0f, 2.0f, 3.0f, 5.0f, 7.0f, 10.0f, 20.0f, 50.0f, 100.0f};

struct Bound{
	float L;
	float U;
		
	Bound(float l, float u) : L(l), U(u) {}
	Bound() : L(0.0f), U(0.0f) {}
};

class IndefiniteFormulasUTest {

public:

  IndefiniteTruthValue* createTV(float l, float u){
  	return new IndefiniteTruthValue(l,u);
  }
  
  IndefiniteTruthValue* createTV(float l, float u, float b){
  		return new IndefiniteTruthValue(l,u,b);
  }
 
  void InferenceTrailPaper(){
 
  	printf("*** Inference Trail Replication Test ***");
  	
		IndefiniteTruthValue::setDefaultConfidenceLevel(0.9);
		IndefiniteTruthValue::setDefaultK(10);
		
        TruthValue* TVin1 = createTV(0.0f,0.138f,0.8f);//inhib1 - A
		TruthValue* TVin0 = createTV(0.0f,0.168f,0.8f);//inhib - B
		TruthValue* TVin2 = createTV(0.0f,0.032f,0.8f);//inhib2 - C
		TruthValue* TVin10 = createTV(0.95f,1.0f,0.9f);//inh inhib1 inhib - AB
		TruthValue* TVin20 = createTV(0.95f,1.0f,0.9f);//inh inhib2 inhib - CB
		
		//TruthValue* TV[5] = {TVin1, TVin0, TVin2, TVin10,TVin20};
		printf("\n\nAdbuction\n");
		TruthValue* TV[5] = {TVin1, TVin0, TVin2, TVin10,TVin20};
		reasoning::IndefiniteSymmetricAbductionFormula fAbduction;
        IndefiniteTruthValue* TVin12=(IndefiniteTruthValue*)(fAbduction.simpleCompute(TV,5));//Inh inhib1 inhib2
		//TruthValue* TVin12=fAbduction.compute(TV);//Inh inhib1 inhib2
		
		printf("\nTVin12:\nL: %.5f - U: %.5f\n",TVin12->getL(),TVin12->getU());
		
		TruthValue* TVA = createTV(0.0, 0.168f,0.8f);
		
		IndefiniteTruthValue* TVin1A = createTV(0.95f,1.0f);
		TruthValue* TV1[1] = {TVin1A};
		printf("\n\nMem2Inh\n");
		reasoning::IndefiniteMem2InhFormula fMem2Inh;
		TVin1A=(IndefiniteTruthValue*)(fMem2Inh.simpleCompute(TV1,1));
		
		printf("\nTVin1A:\nL: %.5f - U: %.5f\n",TVin1A->getL(),TVin1A->getU());		
		
		TruthValue* TV2[3] = {TVin1, TVin2, TVin12};
		printf("\n\nBayes\n");
		reasoning::IndefiniteSymmetricBayesFormula fBayes;
		IndefiniteTruthValue* TVin21=(IndefiniteTruthValue*)(fBayes.simpleCompute(TV2,3));//Inh inhib2 inhib1
		
		printf("\nTVin21:\nL: %.5f - U: %.5f\n",TVin21->getL(),TVin21->getU());		


		TruthValue* TV3[5] = {TVin2, TVin1, TVA, TVin21,TVin1A};
		reasoning::IndefiniteSymmetricDeductionFormula fDeduction;
		IndefiniteTruthValue* TVin2A=(IndefiniteTruthValue*)(fDeduction.simpleCompute(TV3,5));//Inh inhib2 A
					
		printf("\nTVin2A:\nL: %.5f - U: %.5f\n",TVin2A->getL(),TVin2A->getU());
					
//		TruthValue* TVA = createTV(0.0f,0.032f,0.8f);
//		TruthValue* TVC = createTV(0.0f,0.168f,0.8f);
//		TruthValue* TVAmem2inh = createTV(0.58f,0.60f);
//		TruthValue* TVCAmem2inh = createTV(0.313043f,0.513043f);
//		TruthValue* TVBAmem2inh = createTV(0.46666f,0.6f);

		TruthValue* TV4[1] = {TVin2A};
		reasoning::IndefiniteInh2MemFormula fInh2Mem;
		TVin2A=(IndefiniteTruthValue*)(fInh2Mem.simpleCompute(TV4,1));
		
		printf("\nTVin2A:\nL: %.5f - U: %.5f\n",TVin2A->getL(), TVin2A->getU());
		
		TruthValue* TVCausalEvent = createTV(0.0, 0.168f,0.8f);
		TruthValue* TVin0CausalEvent = createTV(0.95, 1.0f,0.9f);
		
		TruthValue* TV5[5] = {TVin2, TVin0, TVCausalEvent, TVin20,TVin0CausalEvent};
//		reasoning::IndefiniteSymmetricDeductionFormula fDeduction;
		IndefiniteTruthValue* TVin2CausalEvent=(IndefiniteTruthValue*)(fDeduction.simpleCompute(TV5,5));//Inh inhib2 causal_event
					
		printf("\nTVin2CausalEvent:\nL: %.5f - U: %.5f\n",TVin2CausalEvent->getL(),TVin2CausalEvent->getU());

		TruthValue* TVPrev = createTV(0.0, 0.044f,0.8f);
		TruthValue* TVPrev1 = createTV(0.0, 0.044f,0.8f);
		TruthValue* TVPrev1Prev = createTV(0.95, 1.0f,0.9f);
		TruthValue* TVPrevCausalEvent = createTV(0.95, 1.0f,0.9f);
		
		TruthValue* TV6[5] = {TVPrev1, TVPrev, TVCausalEvent, TVPrev1Prev, TVPrevCausalEvent};
//		reasoning::IndefiniteSymmetricDeductionFormula fDeduction;
		IndefiniteTruthValue* TVPrev1CausalEvent=(IndefiniteTruthValue*)(fDeduction.simpleCompute(TV6,5));//Inh Prev1 causal_event
					
		printf("\nTVPrev1CausalEvent:\nL: %.5f - U: %.5f\n",TVPrev1CausalEvent->getL(),TVPrev1CausalEvent->getU());
  }
  
  void SymmetricAbductionFormula(){
		
		printf("** Abduction Formula **\n");
		IndefiniteTruthValue::setDefaultConfidenceLevel(0.9);
		IndefiniteTruthValue::setDefaultK(10);
		float LA=0.0f;
		float LB=0.0f;
		float LC=0.0f;
		float LAB=0.95f;
		float LCB=0.95f;
		float UA=0.138f;
		float UB=0.168f;
		float UC=0.032f;
		float UAB=1.0f;		
		float UCB=1.0f;
		float expectedL=0.00003f;
		float expectedU=0.16866f;
		float expectedMean=0.081f;
		float error=0.05;


		TruthValue* TVA = createTV(LA,UA);
		TruthValue* TVB = createTV(LB,UB);
		TruthValue* TVC = createTV(LC,UC);
		TruthValue* TVAB = createTV(LAB,UAB);
		TruthValue* TVCB = createTV(LCB,UCB);

/****** Other premises - test may fail **********************/
/*		
		TruthValue* TVA = createTV(0.0f,0.138f,0.8f);
		TruthValue* TVB = createTV(0.0f,0.168f,0.8f);
		TruthValue* TVC = createTV(0.0f,0.032f,0.8f);
		TruthValue* TVAB = createTV(0.95f,1.0f,0.9f);
		TruthValue* TVCB = createTV(0.95f,1.0f,0.9f);
*/
/*************************************************************/

		TruthValue* TV[5] = {TVA, TVB, TVC, TVAB,TVCB};
		reasoning::IndefiniteSymmetricAbductionFormula fAbduction;

		//unsigned long time_start=getElapsedMillis();
		IndefiniteTruthValue* result=(IndefiniteTruthValue*)(fAbduction.simpleCompute(TV,5));
		//unsigned long time_end=getElapsedMillis();
		
		printf("\nResult:\nL: %.5f - U: %.5f\n",result->getL(),result->getU());
		printf("Mean: %.3f\n\n",result->getMean());
//		printf("Time: %lu\n",time_end - time_start);

//		TS_ASSERT(result->getL() >= expectedL-error && result->getL() <= expectedL+error); 
//		TS_ASSERT(result->getU() >= expectedU-error && result->getU() <= expectedU+error);
//		TS_ASSERT(result->getMean() >= expectedMean-error && result->getMean() <= expectedMean+error);
  }
  
  void Mem2InhFormula(){
  	printf("** Mem2Inh Formula **\n");
		
		IndefiniteTruthValue::setDefaultConfidenceLevel(0.9);
		IndefiniteTruthValue::setDefaultK(10);
		float LA=0.95f;
		float UA=1.0f;
		float expectedL=0.93750f;
		float expectedU=1.0f;
		float expectedMean=0.975f;
		float error=0.05;		
		
		TruthValue* TVA = createTV(LA,UA);
		
		TruthValue* TV[1] = {TVA};
		reasoning::IndefiniteMem2InhFormula fMem2Inh;

		//unsigned long time_start=getElapsedMillis();
		IndefiniteTruthValue* result=(IndefiniteTruthValue*)(fMem2Inh.simpleCompute(TV,1));
		//unsigned long time_end=getElapsedMillis();
		
		printf("\nResult:\nL: %.5f - U: %.5f\n",result->getL(),result->getU());
		printf("Mean: %.3f\n\n",result->getMean());
//		printf("Time: %lu\n",time_end - time_start);

//		TS_ASSERT(result->getL() >= expectedL-error && result->getL() <= expectedL+error); 
//		TS_ASSERT(result->getU() >= expectedU-error && result->getU() <= expectedU+error);
//		TS_ASSERT(result->getMean() >= expectedMean-error && result->getMean() <= expectedMean+error);
  }
  
	void SymmetricAndFormula() {
		
		printf("** AND Formula **\n");
		IndefiniteTruthValue::setDefaultConfidenceLevel(0.9);
		IndefiniteTruthValue::setDefaultK(10);
		float LA=0.99f;
		float LB=0.99f;
		float UA=1.0f;
		float UB=1.0f;
		float expectedL=0.984f;
		float expectedU=0.993f;
		float expectedMean=0.990f;
		float error=0.05;
		
/*
		TruthValue* TVA=createTV(0.95f,1.0f);
		TruthValue* TVB=createTV(0.0f,0.138f);
*/
		TruthValue* TVA=createTV(LA,UA);
		TruthValue* TVB=createTV(LB,UB);

		TruthValue* TV[2] = {TVA,TVB};
		reasoning::IndefiniteSymmetricANDFormula fAND;

//		unsigned long time_start=getElapsedMillis();
		IndefiniteTruthValue* result=(IndefiniteTruthValue*)(fAND.simpleCompute(TV,2));
//		unsigned long time_end=getElapsedMillis();
		
		printf("\nResult:\nL: %.5f - U: %.5f\n\n",result->getL(),result->getU());
		printf("Mean: %.3f\n\n",result->getMean());
//		printf("Time: %lu\n",time_end - time_start);
		
//		TS_ASSERT(result->getL() >= expectedL-error && result->getL() <= expectedL+error); 
//		TS_ASSERT(result->getU() >= expectedU-error && result->getU() <= expectedU+error);
//		TS_ASSERT(result->getMean() >= expectedMean-error && result->getMean() <= expectedMean+error);
		
        //return result;
	}
	
	void SymmetricImplicationBreakdownFormula(){
		
		printf("** Implication Formula **\n");
		IndefiniteTruthValue::setDefaultConfidenceLevel(0.9);
		IndefiniteTruthValue::setDefaultK(10);
		float LA=0.99f;
		float LAB=0.99f;
		float UA=1.0f;
		float UAB=1.0f;
		float expectedL=0.97156f;
		float expectedU=0.99950f;
		float expectedMean=0.993f;
		float error=0.05;
		
		TruthValue* TVA=createTV(LA,UA);
		TruthValue* TVAB=createTV(LAB,UAB);

		TruthValue* TV[2] = {TVAB,TVA};
		reasoning::IndefiniteSymmetricImplicationBreakdownFormula fImplication;

//		unsigned long time_start=getElapsedMillis();
		IndefiniteTruthValue* result=(IndefiniteTruthValue*)(fImplication.simpleCompute(TV,2));
//		unsigned long time_end=getElapsedMillis();
		
		printf("\nResult:\nL: %.5f - U: %.5f\n\n",result->getL(),result->getU());
		printf("Mean: %.3f\n\n",result->getMean());
//		printf("Time: %lu\n",time_end - time_start);
		
//		TS_ASSERT(result->getL() >= expectedL-error && result->getL() <= expectedL+error); 
//		TS_ASSERT(result->getU() >= expectedU-error && result->getU() <= expectedU+error);
//		TS_ASSERT(result->getMean() >= expectedMean-error && result->getMean() <= expectedMean+error);
		
		//return result;
		
	}

	void SymmetricRevisionFormula(){
		printf("** Revision Formula **\n");
		IndefiniteTruthValue::setDefaultConfidenceLevel(0.9);
		IndefiniteTruthValue::setDefaultK(10);
		float LA=0.99f;
		float LB=0.99f;
		float UA=1.0f;
		float UB=1.0f;
		float expectedL=0.99017f;
		float expectedU=0.99860f;
		float expectedMean=0.995f;
		float error=0.05;
		
		TruthValue* TVA=createTV(LA,UA);
		TruthValue* TVB=createTV(LB,UB);

		TruthValue* TV[2] = {TVA,TVB};
		reasoning::IndefiniteSymmetricRevisionFormula fRevision;

//		unsigned long time_start=getElapsedMillis();
		IndefiniteTruthValue* result=(IndefiniteTruthValue*)(fRevision.simpleCompute(TV,2));
//		unsigned long time_end=getElapsedMillis();
		
		printf("\nResult:\nL: %.5f - U: %.5f\n\n",result->getL(),result->getU());
		printf("sD: %.3f\n\n",result->getMean());
//		printf("Time: %lu\n",time_end - time_start);
		
//		TS_ASSERT(result->getL() >= expectedL-error && result->getL() <= expectedL+error); 
//		TS_ASSERT(result->getU() >= expectedU-error && result->getU() <= expectedU+error);
//		TS_ASSERT(result->getMean() >= expectedMean-error && result->getMean() <= expectedMean+error);
		
		//return result;
	}	
	
	void SymmetricDeductionFormula(){
		
		printf("** Deduction Formula **\n");
		IndefiniteTruthValue::setDefaultConfidenceLevel(0.9);
		IndefiniteTruthValue::setDefaultK(2);
		reasoning::setSaveDeductionLookupTable(false);
		reasoning::setUseDeductionLookupTable(false);
		
		//DeductionLookupTable::getInstance()->readTable();
		
		float LA=0.434783f;
		float LB=0.44f;
		float LC=0.44f;
		float LAB=0.313043f;
		float LBC=0.46666f;
		float UA=0.531739f;
		float UB=0.46f;
		float UC=0.74f;
		float UAB=0.513043f;		
		float UBC=0.6f;
		float expectedL=0.4502f;
		float expectedU=0.64767f;
		float expectedMean=0.549f;
		float error=0.5;//a lot of error because the lookup table precision


		TruthValue* TVA = createTV(LA,UA);
		TruthValue* TVB = createTV(LB,UB);
		TruthValue* TVC = createTV(LC,UC);
		TruthValue* TVAB = createTV(LAB,UAB);
		TruthValue* TVBC = createTV(LBC,UBC);
	
/*****************************************************************************/
/* Other premises - tests may fail with them, unless one 
 * changes the expected values 
 */
/*	
		TruthValue* TVA = createTV(0.0f,0.032f,0.8f);
		TruthValue* TVB = createTV(0.0f,0.168f,0.8f);
		TruthValue* TVC = createTV(0.58f,0.60f);
		TruthValue* TVAB = createTV(0.313043f,0.513043f);
		TruthValue* TVBC = createTV(0.46666f,0.6f);
*/

/*		
		TruthValue* TVA = createTV(0.6f,0.7f,0.9f);
		TruthValue* TVB = createTV(0.5f,0.7f,0.9f);
		TruthValue* TVC = createTV(0.4f,0.6f,0.9f);
		TruthValue* TVAB = createTV(0.2f,0.3f,0.9f);
		TruthValue* TVBC = createTV(0.5f,0.7f,0.9f);
*/

/*
    TruthValue* TVA = createTV(0.1f,0.3f);
		TruthValue* TVB = createTV(0.1f,0.3f);
		TruthValue* TVC = createTV(0.1f,0.3f);
		TruthValue* TVAB = createTV(0.1f,0.3f);
		TruthValue* TVBC = createTV(0.1f,0.3f);
*/

//inconsistency teste
/*
  	TruthValue* TVA = createTV(0.0f,0.3f);
		TruthValue* TVB = createTV(0.7f,1.0f);
		TruthValue* TVC = createTV(0.0f,0.3f);
		TruthValue* TVAB = createTV(0.0f,0.3f);
		TruthValue* TVBC = createTV(0.7f,1.0f);
*/
/*****************************************************************************/
		TruthValue* TV[5] = {TVA, TVB, TVC, TVAB,TVBC};
		reasoning::IndefiniteSymmetricDeductionFormula fDeduction;
//		unsigned long time_start=getElapsedMillis();		
		IndefiniteTruthValue* result=(IndefiniteTruthValue*)(fDeduction.simpleCompute(TV,5));

//		unsigned long time_end=getElapsedMillis();
		
		printf("\nResult:\nL: %.5f - U: %.5f\n",result->getL(),result->getU());
		printf("Mean: %.3f\n\n",result->getMean());
//		printf("Time: %lu\n",time_end - time_start);
		printf("\n");

//		TS_ASSERT(result->getL() >= expectedL-error && result->getL() <= expectedL+error); 
//		TS_ASSERT(result->getU() >= expectedU-error && result->getU() <= expectedU+error);
//		TS_ASSERT(result->getMean() >= expectedMean-error && result->getMean() <= expectedMean+error);
		
		//return result;
	}

	void SymmetricBayesFormula(){
		
		printf("** Bayes Formula **\n");
		IndefiniteTruthValue::setDefaultConfidenceLevel(0.8);
		IndefiniteTruthValue::setDefaultK(10);
		float LA=0.0f;
		float LC=0.0f;
		float LAC=0.0f;
		float UA=0.138f;
		float UC=0.032f;
		float UAC=0.17975f;		
		float expectedL=0.00078f;
		float expectedU=0.24815f;
		float expectedMean=0.113f;
		float error=0.05;
		
/*		TruthValue* TVA = createTV(0.476190f,0.571428f);//A
		TruthValue* TVC = createTV(0.380952f,0.571428f);//C
		TruthValue* TVAC = createTV(0.06f,0.14f);//AC
		TruthValue* TVRB = createTV(0.06,0.14f);
	*/
		TruthValue* TVA = createTV(LA,UA);//A
		TruthValue* TVC = createTV(LC,UC);//C
		TruthValue* TVAC = createTV(LAC,UAC);//AC

		TruthValue* TV[3] = {TVA, TVC, TVAC};
		reasoning::IndefiniteSymmetricBayesFormula fBayes;

//		unsigned long time_start=getElapsedMillis();
		IndefiniteTruthValue* result=(IndefiniteTruthValue*)(fBayes.simpleCompute(TV,3));
//		unsigned long time_end=getElapsedMillis();
		
		printf("\nResult:\nL: %.5f - U: %.5f\n\n",result->getL(),result->getU());
		printf("Mean: %.3f\n\n",result->getMean());
//		printf("Time: %lu\n",time_end - time_start);

//		TS_ASSERT(result->getL() >= expectedL-error && result->getL() <= expectedL+error); 
//		TS_ASSERT(result->getU() >= expectedU-error && result->getU() <= expectedU+error);
//		TS_ASSERT(result->getMean() >= expectedMean-error && result->getMean() <= expectedMean+error);
	
		printf("\n");

		//return result;		
		
	}
	
	IndefiniteTruthValue* SymmetricDeductionFormula(TruthValue* TVA, 
							       TruthValue* TVB,
								   TruthValue* TVC,
        						   TruthValue* TVAB,
		    				       TruthValue* TVBC)
	{
		
		TruthValue* TV[5] = {TVA, TVB, TVC, TVAB,TVBC};
		reasoning::IndefiniteSymmetricDeductionFormula fDeduction;
//		unsigned long time_start=getElapsedMillis();		
		IndefiniteTruthValue* result=((IndefiniteTruthValue*)fDeduction.simpleCompute(TV,5));
//		unsigned long time_end=getElapsedMillis();
		
//		printf("\nResult:\nL: %.5f - U: %.5f\n",result->getL(),result->getU());
//		printf("sAC: %.3f\n\n",result->getMean());
//		printf("Time: %lu\n",time_end - time_start);	
//		printf("\n");
		
//		delete TVA; delete TVB; delete TVC; delete TVAB; delete TVBC;
		
		return result;
		
	}

	void ReadLookupTable(){
		IndefiniteTruthValue::setDefaultConfidenceLevel(0.9);
		IndefiniteTruthValue::setDefaultK(2.0);
		reasoning::setSaveDeductionLookupTable(false);
		reasoning::setUseDeductionLookupTable(true);
		DeductionLookupTable::getInstance()->readTable();
	}

	void LookupTablePrecision(){
		
		//para i=0.1 até 0.8
		//para j=i+0.1 até 0.9
		//create TVs
		//save lookup true
		//use lookup false
		//compute deduction
		//if result < 0.0 ou > 1.0, continue
		//save lookup false
		//use lookup true
		//compute deduction
		//compare results (accuracy) (abs)

		IndefiniteTruthValue::setDefaultConfidenceLevel(0.9);
		IndefiniteTruthValue::setDefaultK(2);
		
		TruthValue* TVA;
		TruthValue* TVB;
		TruthValue* TVC;
		TruthValue* TVAB;
		TruthValue* TVBC;
		IndefiniteTruthValue* result;
		IndefiniteTruthValue* resultLookupTable;
		float default_b=0.9f;
		
		vector<float> results;
		
//		DeductionLookupTable::getInstance()->readTable();
//	float Ls[]={0.0f,0.1f,0.2f,0.3f,0.4f,0.5f,0.6f,0.7f,0.8f,0.9f};
//		float Us[]={0.1f,0.2f,0.3f,0.7f,0.5f,0.6f,0.7f,0.8f,0.9f,1.0f};

		float Ls[]={0.0001f,0.3001f,0.5001f,0.7001f};
		float Us[]={0.3001f,0.5001f,0.7001f,0.9999f};//100000 opcoes
		
		int size=4;

		bool useTable=false;

		reasoning::setSaveDeductionLookupTable(false);
		reasoning::setUseDeductionLookupTable(useTable);

		if(useTable)
			DeductionLookupTable::getInstance()->readTable();

		for(int i=0; i<size; i++)
		{
			for(int j=i; j<size; j++)
			{
				printf("[i,j]=[%.3f,%.3f]\n",Ls[i],Us[j]);
				TVA = createTV(Ls[i],Us[j],default_b);
				TVB = createTV(Ls[i],Us[j],default_b);
				TVC = createTV(Ls[i],Us[j],default_b);
				TVAB = createTV(Ls[i],Us[j],default_b);
				TVBC = createTV(Ls[i],Us[j],default_b);
							
				result=(IndefiniteTruthValue*)SymmetricDeductionFormula(TVA,TVB,TVC, TVAB, TVBC);
				printf("[%.5f, %.5f]\n",result->getL(),result->getU());
				printf("Result Mean= %.3f\n\n",result->getMean());
//				resultsLookupTable.push_back(resultLookupTable->getMean());
			}
		}

//		for(int k=0; k<results.size(); k++){
//				float accuracy=(1-((result->getMean()-resultLookupTable->getMean())/result->getMean()))*100;
//				printf("MeanResult = %.3f\n",results[k]);
//				printf("MeanResultLookup = %.3f\n",resultsLookupTable[k]);
//				printf("Accuracy = %.3f \%\n",accuracy);
//		}
				
	}
	
	void testCreateLookupTable(){
		IndefiniteTruthValue::setDefaultConfidenceLevel(0.9);
		IndefiniteTruthValue::setDefaultK(2.0);
		
//		DeductionLookupTable::getInstance()->readTable();		
		
		float default_b=0.9f;
		
		reasoning::setSaveDeductionLookupTable(true);
		reasoning::setUseDeductionLookupTable(false);
//		unsigned long time_start=getElapsedMillis();
		//DeductionLookupTable::getInstance()->readTable();
		reasoning::BuildLookuptTableIndefiniteSymmetricDeductionFormula fBuildLookupDeduction;
		fBuildLookupDeduction.compute();
//        unsigned long time_end=getElapsedMillis();
//        printf("Time: %lu\n",time_end - time_start);
	}

    void testEvaluateDeductionLookupTable(){
        
    }
};

int main(int argc, char *argv[])
{
	/*
	u_int64_t tps = 0;	 TSC ticks per second
	struct timeval tv_before, tv_after;
	tscreg_t tsc_before, tsc_after;
	tscreg_t tsc_begin, tsc_end;

	gettimeofday(&tv_before, NULL);
	get_tsc(&tsc_before );
	sleep(CALIBRATION_INTERVAL);
	gettimeofday(&tv_after, NULL);
	get_tsc(&tsc_after);
	tps = (tsc_after.ll - tsc_before.ll)*1000000ULL /
		((tv_after.tv_sec - tv_before.tv_sec)*1000000ULL +
		 (tv_after.tv_usec - tv_before.tv_usec));

	//get_tsc(&tsc_begin);
	gettimeofday(&tv_before, NULL);
	for(int a=1; a<=100000000; a++);
	gettimeofday(&tv_after, NULL);
	//get_tsc(&tsc_end);
	int64_t result = 0;

	result = ((tv_after.tv_sec - tv_before.tv_sec)*1000000ULL +
		 (tv_after.tv_usec - tv_before.tv_usec));

//	result = (tsc_end.ll - tsc_begin.ll) * 1000000ULL;
//	result /= tps;
	
	cout <<"\n\n1 sec is "<<((int64_t) tps) << " instructions";
	cout <<"\n\nLoop of 10 mi took "<< result << " usecs.";

	get_tsc(&tsc_begin);
	for(int a=1; a<=100000000; a++);
	get_tsc(&tsc_end);
	result = 0;
	result = (tsc_end.ll - tsc_begin.ll) * 1000000ULL;
	result /= 2*tps;

	cout <<"\n\nLoop of 10 mi took "<< result << " usecs.\n\n";
	*/
	
/*clock_t start,finish;
double time;
start = clock();
for(int a=1; a<=1000000; a++);
finish = clock();

time = (double(finish)-double(start));
cout<<"\n\nIt took "<<time<<" ticks.";
*/

/*     int elapseTicks;
     double elapseMilli, elapseSeconds, elapseMinutes;
     clock_t start, stop;

     start = clock() * CLK_TCK;
     for(int a=1; a<=1000000; a++);
     stop = clock() * CLK_TCK;
     
     elapseTicks = stop - start;        //the number of ticks from Begin to End
     elapseMilli = elapseTicks/1000;     //milliseconds from Begin to End
     elapseSeconds = elapseMilli/1000;   //seconds from Begin to End
     elapseMinutes = elapseSeconds/60;   //minutes from Begin to End
          
	cout<<"\n\nIt took "<<elapseTicks<<" ticks.";
     if(elapseSeconds < 1)
          cout<<"\n\nIt took "<<elapseMilli<<" milliseconds.";
     else if(elapseSeconds == 1)
          cout<<"\n\nIt took  1 second.";
     else if(elapseSeconds > 1 && elapseSeconds < 60)
          cout<<"\n\nIt took  "<<elapseSeconds<<" seconds.";
     else if(elapseSeconds >= 60)     
          cout<<"\n\nIt took  "<<elapseMinutes<<" minutes.";
*/
    // check command line
	IndefiniteFormulasUTest *test = new IndefiniteFormulasUTest();
	//test->testCreateLookupTable();
	
	//test->InferenceTrailPaper();
	//test->Mem2InhFormula();

	//test->SymmetricAndFormula();
	//test->SymmetricImplicationBreakdownFormula();
	//test->SymmetricRevisionFormula();
	//test->SymmetricBayesFormula();
	//test->SymmetricAbductionFormula();
	//test->SymmetricDeductionFormula();
}
