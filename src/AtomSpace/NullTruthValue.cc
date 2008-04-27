/**
* NullTruthValue.cc
*
* @author Welter Silva
*/

#include "NullTruthValue.h"
#include "exceptions.h"

NullTruthValue::NullTruthValue() {};

bool NullTruthValue::isNullTv() const { 
    return true; 
}

bool NullTruthValue::operator==(const TruthValue& rhs) const
{
    const NullTruthValue *ntv = dynamic_cast<const NullTruthValue *>(&rhs);
    if (ntv) return true;
    return false;
}

std::string NullTruthValue::toString() const { 
    return "(null TV)"; 
}

float NullTruthValue::getMean() const throw (RuntimeException) { 
    throw RuntimeException(TRACE_INFO, "Cannot call getMean() method of a NullTruthvalue"); 
}

float NullTruthValue::getCount() const throw (RuntimeException) { 
    throw RuntimeException(TRACE_INFO, "Cannot call getCount() method of a NullTruthvalue"); 
}

float NullTruthValue::getConfidence() const throw (RuntimeException)  { 
    throw RuntimeException(TRACE_INFO, "Cannot call getConfidence() method of a NullTruthvalue"); 
}

float NullTruthValue::toFloat() const throw (RuntimeException) { 
    throw RuntimeException(TRACE_INFO, "Cannot call toFloat() method of a NullTruthvalue"); 
}

TruthValueType NullTruthValue::getType() const throw (RuntimeException) { 
    throw RuntimeException(TRACE_INFO, "Cannot call getType() method of a NullTruthvalue"); 
}

TruthValue* NullTruthValue::merge(TruthValue*) throw (RuntimeException) { 
    throw RuntimeException(TRACE_INFO, "Cannot call merge() method of a NullTruthvalue"); 
}

TruthValue* NullTruthValue::clone() const throw (RuntimeException) { 
    throw RuntimeException(TRACE_INFO, "Cannot call clone() method of a NullTruthvalue"); 
}

NullTruthValue& NullTruthValue::operator=(const TruthValue& rhs) throw (RuntimeException) { 
    throw RuntimeException(TRACE_INFO, "Cannot call operator= of a NullTruthvalue"); 
}

