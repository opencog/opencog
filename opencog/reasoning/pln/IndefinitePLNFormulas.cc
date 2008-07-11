#include "IndefinitePLNFormulas.h"

gsl_rng* rng_ = gsl_rng_alloc (gsl_rng_mt19937);

/// Formula defined in the integral of step one [(x-L1)^ks * (U1-x)^k(1-s)
double integralFormula (double x, void * params)
{
  double L_, U_;///L1 and U1
  double k_, s_;
  
  double *in_params = static_cast<double*>(params);
  L_ = in_params[0];
  U_ = in_params[1];
  k_ = in_params[2];
  s_ = in_params[3];
  
  double f = (pow((x-L_),(k_*s_))) * pow((U_-x),(k_*(1-s_)));
  return f;
}

/**
 * Compute the integral of (x-L1)^ks * (U1-x)^k(1-s) between lowerBound and upperBound, 
 * which is in numerator and denominator of the step 1 formula (compute diff) 
 */
float computeDensityFunctionIntegral(float lowerBound,
																			float upperBound,
																			float L_,
																			float U_,
																			float k_,
																			float s_)
{
	double params [4];
  params[0] = L_;
  params[1] = U_;
  params[2] = k_;
  params[3] = s_;

  gsl_function F;
  F.function = &integralFormula;
  F.params = &params;

  int status = 0; size_t neval = 0;
  double result = 0, abserr = 0 ;
  
  status = gsl_integration_qng (&F, lowerBound, upperBound, 1e-1, 0.0,
				  &result, &abserr, &neval) ;
				  
	return (float)result;
  
}

/**
 * This method is responsiple of computing the L1 and U1 of step 1. L1 = L-diff and
 * U1=U+diff
 */
float computeDiff(IndefiniteTruthValue* TV)
{
	float L=TV->getL();
	float U=TV->getU();
	float L_,U_;
	
//  if(diff==0.0f)diff=0.01f;
	float diff = 0.01;

  float expected_result = (1-TV->getConfidenceLevel())/2;

  float result=0.0, result_denominator=0.0, result_numerator=0.0;
  float u,l;
  ///while the result is not according to the accepted error, try another diff value using
  ///binary search
  while((result < expected_result-reasoning::diffError) || (result > expected_result+reasoning::diffError) ) 
  {
 		u = U+diff;
  	l = L-diff;
  	U_=u;
//		U_=(u > 1.0)?1.0:u;
	  L_=l;
//  	L_=(l < 0.0)?L_=0.0:l;

//  	result_numerator = computeDensityFunctionIntegral(L_,L,L_,U_,IndefiniteTruthValue::DEFAULT_K,reasoning::s);
  	result_numerator = computeDensityFunctionIntegral(U,U_,L_,U_,IndefiniteTruthValue::DEFAULT_K,reasoning::s);
  	result_denominator = computeDensityFunctionIntegral(L_,U_,L_,U_,IndefiniteTruthValue::DEFAULT_K,reasoning::s);
	
  	result=result_numerator/result_denominator;  	
  	
  	///binary search for diff
  	if(result < expected_result-reasoning::diffError)
  	{
    	diff = diff + diff/2;
  	}else
  	{
    	if(result > expected_result+reasoning::diffError)
    	{
      	diff = diff - diff/2;
    	}
  	}
	//printf("diff= %.3f result= %.3f U= %.3f L= %.3f\n", diff, result, U, L);
  }
 	  
  TV->setDiff(diff);
  return diff;
}

float generateRandomValueBetaDistribution(double a, double b)
{
  return (float)(gsl_ran_beta(rng_,a,b));
}

/**
 * Use GSL to generate n random values of beta distribution
 * @param values the result
 * @param n the number of random values
 * @param a alfa parameter of beta distribution
 * @param b beta parameter of beta distribution
 * 
 */
void generateRandomValueBetaDistribution(float values[], int n, double a, double b)
{
	for(int i=0; i < n; i++)
	{
		values[i]=generateRandomValueBetaDistribution(a,b);
		//printf("values[i] = %03f\n", values[i]);
	}
}

float scaleRandomValue(float value, float L_, float U_)
{
	value=L_+(U_-L_)*value;
	value=(value > 1.0)?1.0:(value < 0.0)?0.0:value;
	return value;
}

/**
 * This method is related to step 2.2. All values are scaled according to
 * the formula: values[i]=L_+(U_-L_)*values[i]. Remembering that values[i]
 * must be between 0.0 and 1.0. Otherwise, it is truncated.
 */
void scaleRandomValue(float values[], int size, float L_, float U_)
{
	for(int i=0; i<size; i++)
	{
		values[i]=scaleRandomValue(values[i],L_,U_);
	}
}

/**
 * @param L_ L_=L-DIFF (L1)
 * @param U_ U_=U-DIFF (U1)
 * @param result the matrix containing the first order distributions of the truth value
 * 
 */
void generateFirstOrderDistributions(IndefiniteTruthValue* TV)
{
	vector<float*> result;
	float L_=TV->getL_();
	float U_=TV->getU_();
	float values[100];///n1 random values scaled in [L_,U_]
	///step 2.1 of Matt doc. The parameters alfa=beta=0.5 is the defaulf for beta distribution 
	float alpha=IndefiniteTruthValue::DEFAULT_K*0.5;
	float beta=alpha;
	generateRandomValueBetaDistribution(values, reasoning::n1, alpha, beta);
	//scale the values to inteval [L_,U_] (step 2.2 of Matt doc		
	scaleRandomValue(values, reasoning::n1, L_, U_);
	
	float values2[100];
	///Step 3
	for(int i=0;i<reasoning::n1;i++)
	{
		truncate(values[i]);
		///The alfa and beta parameters are setted with another values defined by Matt/Ben
		generateRandomValueBetaDistribution(values2, reasoning::n2, IndefiniteTruthValue::DEFAULT_K*values[i], IndefiniteTruthValue::DEFAULT_K*(1-values[i]));
		result.push_back(new float[reasoning::n2]);
		for(int j=0;j<reasoning::n2;j++)
		{
			truncate(values2[j]);
			result[i][j]=values2[j];
			//printf("sampling %03f\n", result[i][j]);
		}
	}
	TV->setFirstOrderDistribution(result);
}



/**
 * This is the last step of the procedure. After the rule computation, the
 * conclusion has to be found.
 * For each first order distribution (n1), it is found the mean value averaging the n2 values of the
 * distribution. The average mean (s) is computed averaging over all n1 elements of the means list.
 * 
 */
IndefiniteTruthValue* findConclusion(vector<float> means, float b)
{
		IndefiniteTruthValue* result=new IndefiniteTruthValue();
		
		int size=means.size();
		
		float strength=0.0;
		float percentile=(1-b)/2;
		double quantityBelow_d=ceil(size*percentile);
		int quantityBelow=int(quantityBelow_d);
		int quantityAbove=quantityBelow;

		for(int i=0; i<size; i++)
		{
			strength += means[i];
		}

		strength=strength/size;
		result->setMean(strength);

		/**
		 * As the list is sorted and I know that qBelow elements must be below L
		 * and qAbove must be above U (computed by the percentile (1-b)/2), then
		 * L will be the element in index qBelow and U the one in index n1-qAbove-1.
		 */
		sort(means.begin(), means.end());
		result->setL(means[quantityBelow]);
		result->setU(means[size-quantityAbove-1]);
/*
		printf("Means\n");
		for(int i=0; i<size; i++)
		{
			printf("%.3f - ",means[i]); 
		}
		printf("\n\n");	
*/
		
		return result;
		
}

vector<float> computeMeans(vector<vector<float> > distribution)
{
	vector<float> means;
	float mean;
	for(int i=0; i<reasoning::n1; i++)
	{
		mean=0.0;
		for(int j=0; j<reasoning::n2; j++)
		{
			mean += distribution[i][j];
		}
		means.push_back((float)(mean/reasoning::n2));
	}
	return means;
}

void truncate(float &value){
	value=(value<=0.0)?0.000001:(value>=1.0)?0.999999:value;
}
