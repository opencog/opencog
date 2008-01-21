/**
* TruthValue.cc
*
* @author Guilherme Lamacie
* @author Welter Silva
*/
#include "TruthValue.h"
#include "SimpleTruthValue.h"
#include "IndefiniteTruthValue.h"
#include "CompositeTruthValue.h"
#include "NullTruthValue.h"
#include <stdio.h>
#include <stdlib.h>
#include <typeinfo>
#include "ClassServer.h"

// Static references to special TVs: 

//const TruthValue& TruthValue::NULL_TV() = NullTruthValue();
const TruthValue& TruthValue::NULL_TV() {
	static TruthValue* instance	= new NullTruthValue();
	return *instance;
}

//const TruthValue& TruthValue::DEFAULT_TV() = SimpleTruthValue(0,0);
const TruthValue& TruthValue::DEFAULT_TV() {
	static TruthValue* instance	= new SimpleTruthValue(0,0);
	return *instance;
}
//const TruthValue& TruthValue::TRUE_TV() = SimpleTruthValue(MAX_TRUTH, MAX_TV_COUNT);
const TruthValue& TruthValue::TRUE_TV() {
	static TruthValue* instance	= new SimpleTruthValue(MAX_TRUTH, MAX_TV_COUNT);
	return *instance;
}

//const TruthValue& TruthValue::FALSE_TV() = SimpleTruthValue(0.0f, MAX_TV_COUNT);
const TruthValue& TruthValue::FALSE_TV() {
	static TruthValue* instance	= new SimpleTruthValue(0.0f, MAX_TV_COUNT);
	return *instance;
}

//const TruthValue& TruthValue::TRIVIAL_TV() = SimpleTruthValue(MAX_TRUTH, 0.0f);
const TruthValue& TruthValue::TRIVIAL_TV() {
	static TruthValue* instance	= new SimpleTruthValue(MAX_TRUTH, 0.0f);
	return *instance;
}

TruthValue& TruthValue::operator=(const TruthValue& rhs) {
    return *this;
}

// VIRTUAL METHODS: 

TruthValue* TruthValue::merge(const TruthValue& other) const {
#if 1  
    // TODO: Use the approach with dynamic cast bellow if we're going to have subclasses 
    // of CompositeTruthValue. For now, this approach using getType() is more efficient.
    if (other.getType() == COMPOSITE_TRUTH_VALUE) {
#else        
    const CompositeTruthValue *otherCTv = dynamic_cast<const CompositeTruthValue *>(&other);
    if (otherCTv) {
#endif        
        return other.merge(*this);
    } else if (other.getConfidence() > getConfidence()) {
        return other.clone();
    }
    return clone();
}

bool TruthValue::isNullTv() const { 
    return false; 
}

// STATIC METHODS:

const char* TruthValue::typeToStr(TruthValueType t) throw (InvalidParamException) {
    switch(t) {
        case SIMPLE_TRUTH_VALUE: 
            return "SIMPLE_TRUTH_VALUE"; 
        case INDEFINITE_TRUTH_VALUE: 
            return "INDEFINITE_TRUTH_VALUE";
        case COMPOSITE_TRUTH_VALUE: 
            return "COMPOSITE_TRUTH_VALUE";
        default: 
           throw InvalidParamException(TRACE_INFO, 
                    "TruthValue - Invalid Truth Value type: '%d'.", t);
    }
}

TruthValueType TruthValue::strToType(const char* str) throw (InvalidParamException) {
    //printf("TruthValue::strToType(%s)\n", str);
    TruthValueType t = (TruthValueType) 0;
    
    while (t != NUMBER_OF_TRUTH_VALUE_TYPES) {
        if (!strcmp(str, typeToStr(t))) {
            return t;
        }
        t = (TruthValueType)((int)t + 1);
    }

    throw InvalidParamException(TRACE_INFO, 
            "TruthValue - Invalid Truth Value type string: '%s'.", str);
}

// Factories: 

TruthValue* TruthValue::factory(const char* fullTvStr) {
    //printf("TruthValue::factory(): fullTvStr = %s\n", fullTvStr);
    char typeStr[1<<16];
    sscanf(fullTvStr,"%s",typeStr);
    TruthValueType type = strToType(typeStr);
    return factory(type, fullTvStr+strlen(typeStr)+1);
}

TruthValue* TruthValue::factory(TruthValueType type, const char* tvStr) throw (InvalidParamException){
    //printf("TruthValue::factory(): type = %s tvStr = %s\n", TruthValue::typeToStr(type), tvStr);
    switch (type) {
        case SIMPLE_TRUTH_VALUE:
            return SimpleTruthValue::fromString(tvStr);
            break;
        case INDEFINITE_TRUTH_VALUE:
            return IndefiniteTruthValue::fromString(tvStr);
            break;
        case COMPOSITE_TRUTH_VALUE:
            return CompositeTruthValue::fromString(tvStr);
            break;
        default: 
            throw InvalidParamException(TRACE_INFO, 
                    "TruthValue - Invalid Truth Value type in factory(...): '%d'.", type);
            break;
    }
    return NULL;
}


void TruthValue::DeleteAndSetDefaultTVIfPertinent(TruthValue** tv) {
    if (*tv != &(TruthValue::DEFAULT_TV()) &&
        (*tv)->getType() == DEFAULT_TV().getType() && 
        (*tv)->getMean() == DEFAULT_TV().getMean() && 
        (*tv)->getCount() == DEFAULT_TV().getCount()) {
        delete *tv;
        *tv = (TruthValue*) &(DEFAULT_TV());
    }
} 
