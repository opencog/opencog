/**
* SimpleTruthValue.cc
*
* @author Guilherme Lamacie
* @author Welter Silva
*/
#include "SimpleTruthValue.h"
#include "exceptions.h"
#include <math.h>

#define KKK 800.0f

SimpleTruthValue::SimpleTruthValue(float m, float c){
    mean = m;
    count = c;
}

SimpleTruthValue::SimpleTruthValue(SimpleTruthValue const& source) {
    mean = source.mean;
    count = source.count;
}

void SimpleTruthValue::setMean(float m){
    mean = m;
}

void SimpleTruthValue::setCount(float c){
    count = c;
}

void SimpleTruthValue::setConfidence(float c){
    count = confidenceToCount(c);
}

float SimpleTruthValue::getMean() const{
    return mean;
}

float SimpleTruthValue::getCount() const{
    return count;
}

float SimpleTruthValue::getConfidence() const{
    return countToConfidence(count);
}

float SimpleTruthValue::toFloat() const{
    return getMean();
}

std::string SimpleTruthValue::toString() const{
    char buf[1024];
    // TODO: confidence is not needed for Saving&Loading. So, for saving memory space 
    // in dump files, it should be removed. However, toString is being used for debug
    // purposes...
    sprintf(buf, "[%f,%f=%f]", getMean(), getCount(),getConfidence());
    return buf;
}

SimpleTruthValue* SimpleTruthValue::clone() const {
    return new SimpleTruthValue(*this);
}

SimpleTruthValue& SimpleTruthValue::operator=(const TruthValue& rhs) throw (RuntimeException) {
    const SimpleTruthValue* tv = dynamic_cast<const SimpleTruthValue*>(&rhs);
    if (tv) {
        if (tv != this) { // check if this is the same object first.
            mean = tv->mean;
            count = tv->count;
        }
    } else {
#if 0
		// The following line was causing a compilation error on MSVC...
        throw RuntimeException(TRACE_INFO, "Cannot assign a TV of type '%s' to one of type '%s'\n", 
                                    typeid(rhs).name(), typeid(*this).name());
#else
        throw RuntimeException(TRACE_INFO, 
                "SimpleTruthValue - Invalid assignment of a SimpleTV object.");
#endif

    }
    return *this;
}

TruthValueType SimpleTruthValue::getType() const{
    return SIMPLE_TRUTH_VALUE;
}

float SimpleTruthValue::confidenceToCount(float c) {
	c = min(c, 0.9999999f);
	return KKK*c/(1.0-c);
}

float SimpleTruthValue::countToConfidence(float c) {
    return (c/(c+KKK));
}

SimpleTruthValue* SimpleTruthValue::fromString(const char* tvStr) {
    float mean, count, conf;
    // TODO: confidence is not needed for Saving&Loading. So, for saving memory space 
    // in dump files, it should be removed. However, toString is being used for debug
    // purposes...
    sscanf(tvStr, "[%f,%f=%f]", &mean, &count,&conf);
    //printf("SimpleTruthValue::fromString(%s) => mean = %f, count = %f, conf = %f\n", tvStr, mean, count, conf);
    return new SimpleTruthValue(mean, count);
}
