/**
* VersionHandle.cc
*
* @author Welter Silva
*/

#include "VersionHandle.h" 
#include "exceptions.h"
#include "TLB.h"

VersionHandle::VersionHandle() {
    indicator = UNKNOWN;
    substantive = UNDEFINED_HANDLE;
}

VersionHandle::VersionHandle(IndicatorType ind, Handle subs) {
    indicator = ind;
    substantive = subs;
}

const char* VersionHandle::indicatorToStr(IndicatorType indicator) throw (InvalidParamException) {
    switch (indicator) {
        case HYPOTHETICAL: 
            return "HYPOTHETICAL";            
        case CONTEXTUAL: 
            return "CONTEXTUAL";            
        case UNKNOWN: 
            return "UNKNOWN";            
        default: 
            throw InvalidParamException(TRACE_INFO, 
                    "VersionHandle - Invalid indicator type: '%d'.", indicator);
    }
}

IndicatorType VersionHandle::strToIndicator(const char* indicatorStr) throw (InvalidParamException){
    for (int i = 0; i <= UNKNOWN; i++) {
        IndicatorType indicator = (IndicatorType) i;
        if (!strcmp(indicatorToStr(indicator), indicatorStr)) {
            return indicator;
        }
    }
    throw InvalidParamException(TRACE_INFO, 
            "VersionHandle - Invalid IndicatorType name: '%s'.", indicatorStr);
}

int hashVersionHandle::operator()(VersionHandle vh) const{
int hashCode =  vh.indicator + hashHandle()(vh.substantive);
    return(hashCode);
}

bool eqVersionHandle::operator()(VersionHandle vh1, VersionHandle vh2) const{
    return (vh1.indicator == vh2.indicator && 
        !CoreUtils::compare(vh1.substantive, vh2.substantive));
}

