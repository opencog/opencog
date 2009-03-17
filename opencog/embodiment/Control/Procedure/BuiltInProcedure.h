#ifndef BUILTINPROCEDURE_H_
#define BUILTINPROCEDURE_H_
/**
 * Base class for BuiltIn Procedures
 * @author Welter Luigi
 * 
 */
#include "GeneralProcedure.h" 
#include <ComboReduct/combo/vertex.h>
#include <list> 

namespace Procedure {
    
class BuiltInProcedure : public GeneralProcedure {

protected:

    unsigned int minArity;
    unsigned int optionalArity;

public: 
    virtual ~BuiltInProcedure(){}
      
    virtual combo::vertex execute(const std::vector<combo::vertex>& arguments) const=0;
    
    ProcedureType getType() const {
        return BUILT_IN;
    }

    /**
     * Indicates if this procedure is an Pet action schemata. 
     * If so, its execute method always return the ActionPlanId for the action sent to Proxy
     */ 
    virtual bool isPetAction() const {
        return false;
    }
    
    /**
     * Return the mandatory arity for the builtin action
     */ 
    unsigned int getArity() const { return minArity; }

    /**
     * Return the optional arity for the buitlin action
     */
    unsigned int getOptionalArity() { return optionalArity; }
    
    /**
     * Return the max arity (min + optional arities) for the builtin action
     */
    unsigned int getMaxArity() const { return (minArity + optionalArity); }
};

} 

#endif /*BUILTINPROCEDURE_H_*/
