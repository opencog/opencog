#ifndef _PREDICATEEVALUATOR_H_
#define _PREDICATEEVALUATOR_H_

#include "types.h"

class PredicateEvaluator {
    
public:     
    virtual ~PredicateEvaluator();
    virtual bool evaluate(Handle h) = 0;
};
#endif //_PREDICATEEVALUATOR_H_
