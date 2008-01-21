/**
* SimpleTruthValue.h
*
* @author Guilherme Lamacie
* @author Murilo Saraiva de Queiroz
* @author Welter Silva
*/

#ifndef _SIMPLE_TRUTH_VALUE_H_
#define _SIMPLE_TRUTH_VALUE_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "TruthValue.h"
#include "utils.h"

class SimpleTruthValue : public TruthValue {

    protected:

        float mean;
        float count;

        void init(float mean,float count); 
        
    public:

        SimpleTruthValue(float mean,float count);
        SimpleTruthValue(SimpleTruthValue const&);

        SimpleTruthValue* clone() const;
        SimpleTruthValue& operator=(const TruthValue& rhs) throw (RuntimeException);

        static SimpleTruthValue* fromString(const char*);
        static float confidenceToCount(float);
        static float countToConfidence(float);

	    float toFloat() const;
        std::string toString() const;
        TruthValueType getType() const;

        float getMean() const;
        float getCount() const;
        float getConfidence() const;
        void setMean(float);
        void setCount(float);
        void setConfidence(float);
};

#endif
