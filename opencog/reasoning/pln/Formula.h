#ifndef _INFERENCE_FORMULA
#define _INFERENCE_FORMULA

#include <iostream>
#include <cstdio>
#include <string>
#include <TruthValue.h>
#include <SimpleTruthValue.h>
#include <IndefiniteTruthValue.h>
#include <CompositeTruthValue.h>
#include <exceptions.h>

#define LIMIT_TRUTH_VALUES 1
#define IS_NAN(f) (!((f)==(f)))

#define DefaultU 10000

using namespace opencog;

namespace reasoning {

template<typename TVType, typename ResultType>
class ArityFreeFormula
{
	public:

    /** You should always provide the N argument to prove the compiler
		that you know how many args this method takes. */

    /***
     * One or more of the following member functions should be 
     * overridden in deriving classes; the general form is:
     *
     *   Result compute(TruthValue* tv1,...,TruthValue* tvN) const;
     *
     * for 1<=N<=4. For larger values, the general N-arry case is:
     *
     *   Result compute(TruthValue** tvs,int N) const;
     ***/

	virtual ResultType compute(TVType**, int N, long U = DefaultU) const {};

	/**
		U is the universe size...
	*/

    TruthValue* compute(TruthValue** TVsub,int Nsub, 
				TruthValue** TVsuper,int Nsuper, long U = DefaultU) const
    {
			return compute(TVsub, Nsub, U);
    }
    
    virtual ResultType compute(TVType* tv1, long U) const {
			return compute(&tv1,1,U);
    }
    virtual ResultType compute(TVType* tv1,TVType *tv2, long U) const {
			static TVType* buffer[2];
			buffer[0]=tv1; buffer[1]=tv2;
			return compute(buffer,2,U);
    }
    virtual ResultType compute(TVType* tv1,TVType* tv2,
		TVType* tv3, long U) const {
			static TVType* buffer[3];
			buffer[0]=tv1; buffer[1]=tv2; buffer[2]=tv3;
			return compute(buffer,3,U);
    }
    virtual ResultType compute(TVType* tv1,TVType* tv2,
		TVType* tv3,TVType* tv4, long U) const {
			static TVType* buffer[4];
			buffer[0]=tv1; buffer[1]=tv2; buffer[2]=tv3; buffer[3]=tv4;
			return compute(buffer,4,U);
    }
	
	public:

	///a simple error hanlder; may be overridden
	virtual void handleError(const std::string& error) const {
		cerr << "ERROR: " << error << endl;
		exit(1);
	}

	virtual ~ArityFreeFormula() {};

    ResultType checkTruthValue(ResultType tv) const
	{
		//if (Log::getDefaultLevel() >= 3) printf("\nRaw result: %f", tv->getMean());
		
		if (LIMIT_TRUTH_VALUES) {
			if (tv->getMean() < 0) {
                if (tv->getType() == SIMPLE_TRUTH_VALUE) {
                    ((SimpleTruthValue*)tv)->setMean(0);
                } else if (tv->getType() == INDEFINITE_TRUTH_VALUE) {
                    ((IndefiniteTruthValue*)tv)->setMean(0);
                } else {
                    throw new RuntimeException(TRACE_INFO, "Cannot limit truth values of this type of TV\n");
                }
            }
			if (tv->getMean() > 1) {
                if (tv->getType() == SIMPLE_TRUTH_VALUE) {
                    ((SimpleTruthValue*)tv)->setMean(1);
                } else if (tv->getType() == INDEFINITE_TRUTH_VALUE) {
                    ((IndefiniteTruthValue*)tv)->setMean(1);
                } else {
                    throw new RuntimeException(TRACE_INFO, "Cannot limit truth values of this type of TV\n");
                }
            }
			if (tv->getCount() < 0) {
                if (tv->getType() == SIMPLE_TRUTH_VALUE) {
                    ((SimpleTruthValue*)tv)->setCount(0);
                } else {
                    throw new RuntimeException(TRACE_INFO, "Cannot limit truth values of this type of TV\n");
                }
            }
		} else {
			if ((tv->getMean() < 0) || (tv->getMean() > 1) || (tv->getCount() < 0))
				return NULL;
		}
		
		if ((IS_NAN(tv->getMean()))||(IS_NAN(tv->getCount()))) {
			//Mike commented out for gcc comatability: ||(isinf(tv->getMean()))||(isinf(tv->getCount()))) {
			return NULL;
		}
		
		return tv;
	}
    
};


/** @class Formula
	_TVN is the arity.
*/

template<int _TVN>
class Formula : public ArityFreeFormula<TruthValue,TruthValue*>
{
    friend class SubsetEvalFormula;
public:
    const int TVN;
    Formula() : TVN(_TVN) {}
    
    /**
     * Simple compute do not handle CompositeTruthValues. It's called by compute() method, 
     * which handles CTVs.
     */
    virtual TruthValue* simpleCompute(TruthValue** TV,int N, long U = DefaultU) const;

    /**
     * Partition the TV input to N/TVN groups of TVN entries.
     * Calculate the formula for each group.
     * Combine the results into a single array of TVs.
     * The size of the array is N/TVN.
     * Like simpleCompute, this method does not handle CTVs.
     */
    TruthValue** multiCompute(TruthValue** TV, int N, long U = DefaultU) const;

    /**
     * Check if there is any CompositeTruthValue object in the array of TruthValues and 
     * ,if so, computes different results for primaryTVs and for each handle-versioned TVS
     */
    TruthValue* compute(TruthValue** TV, int N, long U = DefaultU) const;

};


} //~namespace
#endif
