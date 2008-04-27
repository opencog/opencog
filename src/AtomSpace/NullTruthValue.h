/**
* NullTruthValue.h
*
* @author Welter Silva
* 
*/

#include "TruthValue.h" 

#ifndef _NULL_TRUTH_VALUE_TV_H_
#define _NULL_TRUTH_VALUE_TV_H_

class NullTruthValue : public TruthValue {

    friend class TruthValue;
    
public:
    bool isNullTv() const;
    float getMean() const throw (RuntimeException);
    float getCount() const throw (RuntimeException);
    float getConfidence() const  throw (RuntimeException);
    float toFloat() const throw (RuntimeException);
    std::string toString() const;
    TruthValueType getType() const throw (RuntimeException);

    virtual bool operator==(const TruthValue& rhs) const;

protected:
    NullTruthValue(); // TODO: Make this constructor protected. For some reason compiler is complaining if it's declared protected.
    TruthValue* merge(TruthValue*) throw (RuntimeException);
    TruthValue* clone() const throw (RuntimeException);
    NullTruthValue& operator=(const TruthValue& rhs) throw (RuntimeException);
    
};

#endif
