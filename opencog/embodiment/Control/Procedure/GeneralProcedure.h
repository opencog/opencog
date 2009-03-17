#ifndef PROCEDURE_H_
#define PROCEDURE_H_
/**
 * Base class for Procedures
 * @author Welter Luigi
 * 
 */
#include <string> 

namespace Procedure {
    
typedef enum {BUILT_IN, COMBO, COMBO_SELECT} ProcedureType; 
    
class GeneralProcedure {

public: 
    virtual ~GeneralProcedure(){}
      
    virtual const std::string& getName() const=0;
    virtual ProcedureType getType() const=0;
    virtual unsigned int getArity() const=0;
};

} 

#endif /*PROCEDURE_H_*/
