/**
 * IndefiniteRuleTest.cc
 */

#include "FormulasIndefinite.h"
#include <math.h>
#include <vector>
#include <ctime>

#define CALIBRATION_INTERVAL 1
using namespace reasoning;

float bvalues[6]={0.5, 0.6, 0.7, 0.8, 0.9, 0.95};
float kvalues[9]={1.0f, 2.0f, 3.0f, 5.0f, 7.0f, 10.0f, 20.0f, 50.0f, 100.0f};

struct Bound{
	float L;
	float U;
		
	Bound(float l, float u) : L(l), U(u) {}
	Bound() : L(0.0f), U(0.0f) {}
};

IndefiniteTruthValue* createTV(float l, float u){
	return new IndefiniteTruthValue(l,u);
}
  
IndefiniteTruthValue* createTV(float l, float u, float b){
	return new IndefiniteTruthValue(l,u,b);
}

void SymmetricAndFormula() {
	printf("** AND Formula **");
	IndefiniteTruthValue::setDefaultConfidenceLevel(0.9);
	IndefiniteTruthValue::setDefaultK(10);
	float LA=0.99f; float LB=0.99f;
	float UA=1.0f; float UB=1.0f;

	IndefiniteTruthValue* TVa=createTV(LA,UA);
	IndefiniteTruthValue* TVb=createTV(LB,UB);
	IndefiniteTruthValue* result;

	RuleGenerator<ConjunctionRule, IndefiniteTruthValue> myCreator;
	ConjunctionRule *a = myCreator.CreateRule(TVa, TVb);
	result = a->solve();
	printf("\nResult:\nL: %.5f - U: %.5f\n",result->getL(),result->getU());
	printf("Mean: %.3f\n",result->getMean());

	printf("\n** AND Formula **");
	
	float L2A=0.051668044685163f; float L2B=0.731203298626716f;
	float U2A=0.264185905611042f; float U2B=0.997599605415954f;

	IndefiniteTruthValue* TV2a=createTV(L2A,U2A);
	IndefiniteTruthValue* TV2b=createTV(L2B,U2B);
	IndefiniteTruthValue* result2;

	ConjunctionRule *a2 = myCreator.CreateRule(TV2a, TV2b);
	result2 = a2->solve();
	printf("\nResult:\nL: %.5f - U: %.5f\n",result2->getL(),result2->getU());
	printf("Mean: %.3f\n",result2->getMean());
}

void SymmetricImplicationBreakdownFormula(){
		
	printf("\n** Implication Formula **");
	IndefiniteTruthValue::setDefaultConfidenceLevel(0.9);
	IndefiniteTruthValue::setDefaultK(10);
	float LA=0.99f;
	float LAB=0.99f;
	float UA=1.0f;
	float UAB=1.0f;

	IndefiniteTruthValue* TVa=createTV(LA,UA);
	IndefiniteTruthValue* TVab=createTV(LAB,UAB);
	IndefiniteTruthValue* result;

	RuleGenerator<ImplicationRule, IndefiniteTruthValue> myCreator;
	ImplicationRule *a = myCreator.CreateRule(TVa, TVab);
	result = a->solve();
	
	printf("\nResult:\nL: %.5f - U: %.5f\n",result->getL(),result->getU());
	printf("Mean: %.3f\n\n",result->getMean());
}

void SymmetricRevisionFormula(){
	printf("** Revision Formula **");
	IndefiniteTruthValue::setDefaultConfidenceLevel(0.9);
	IndefiniteTruthValue::setDefaultK(10);
	float LA=0.99f;
	float LB=0.99f;
	float UA=1.0f;
	float UB=1.0f;

	IndefiniteTruthValue* TVa=createTV(LA,UA);
	IndefiniteTruthValue* TVb=createTV(LB,UB);
	IndefiniteTruthValue* result;

	RuleGenerator<RevisionRule, IndefiniteTruthValue> myCreator;
	RevisionRule *a = myCreator.CreateRule(TVa, TVb);
	result = a->solve();
	
	printf("\nResult:\nL: %.5f - U: %.5f\n",result->getL(),result->getU());
	printf("Mean: %.3f\n\n",result->getMean());
}

void SymmetricBayesFormula(){
	printf("** Bayes Formula **");
	IndefiniteTruthValue::setDefaultConfidenceLevel(0.8);
	IndefiniteTruthValue::setDefaultK(10);
	
	float LA=0.0f;
	float LC=0.0f;
	float LAC=0.0f;
	float UA=0.138f;
	float UC=0.032f;
	float UAC=0.17975f;		
	
	IndefiniteTruthValue* TVa=createTV(LA,UA);
	IndefiniteTruthValue* TVc=createTV(LC,UC);
	IndefiniteTruthValue* TVac=createTV(LAC,UAC);
	IndefiniteTruthValue* result;

	RuleGenerator<BayesRule, IndefiniteTruthValue> myCreator;
	BayesRule *a = myCreator.CreateRule(TVa, TVc, TVac);
	result = a->solve();

	printf("\nResult:\nL: %.5f - U: %.5f\n",result->getL(),result->getU());
	printf("Mean: %.3f\n\n",result->getMean());
}

void SymmetricAbductionFormula(){
		
	printf("** Abduction Formula **");
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

	IndefiniteTruthValue* TVa=createTV(LA,UA);
	IndefiniteTruthValue* TVb=createTV(LB,UB);
	IndefiniteTruthValue* TVc=createTV(LC,UC);
	IndefiniteTruthValue* TVab=createTV(LAB,UAB);
	IndefiniteTruthValue* TVcb=createTV(LCB,UCB);
	IndefiniteTruthValue* result;

	RuleGenerator<AbductionRule, IndefiniteTruthValue> myCreator;
	AbductionRule *a = myCreator.CreateRule(TVa, TVb, TVc, TVab, TVcb);
	result = a->solve();

	printf("\nResult:\nL: %.5f - U: %.5f\n",result->getL(),result->getU());
	printf("Mean: %.3f\n\n",result->getMean());
}

void SymmetricDeductionFormula(){
		
	printf("** Deduction Formula **\n");
	IndefiniteTruthValue::setDefaultConfidenceLevel(0.9);
	IndefiniteTruthValue::setDefaultK(2);

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

	IndefiniteTruthValue* TVa=createTV(LA,UA);
	IndefiniteTruthValue* TVb=createTV(LB,UB);
	IndefiniteTruthValue* TVc=createTV(LC,UC);
	IndefiniteTruthValue* TVab=createTV(LAB,UAB);
	IndefiniteTruthValue* TVbc=createTV(LBC,UBC);
	IndefiniteTruthValue* result;

	RuleGenerator<DeductionRule, IndefiniteTruthValue> myCreator;
	DeductionRule *a = myCreator.CreateRule(TVa, TVb, TVc, TVab, TVbc);
	result = a->solve();

	printf("Result:\nL: %.5f - U: %.5f\n",result->getL(),result->getU());
	printf("Mean: %.3f\n\n",result->getMean());
}

int main(int argc, char *argv[]){
	SymmetricAndFormula();
	SymmetricImplicationBreakdownFormula();
	SymmetricRevisionFormula();
	SymmetricBayesFormula();	
	SymmetricAbductionFormula();
	SymmetricDeductionFormula();
}
