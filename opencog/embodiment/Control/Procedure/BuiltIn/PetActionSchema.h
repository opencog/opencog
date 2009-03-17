#ifndef PET_ATION_SCHEMA_H_
#define PET_ATION_SCHEMA_H_
/**
 * PetActionSchema.h:
 *
 * This is a class for all builtin schema that executes a single Pet action
 * The execute() method always returns the ID of the ActionPlan sent to SL/Proxy so 
 * that Procedure Interpreter can check if the action plan has really finished or failed. 
 * 
 * @author Welter Luigi
 */

#include "BuiltInProcedure.h"
#include "PAI.h"

#include <exception>

namespace Procedure { 

class PetActionSchema : public BuiltInProcedure {

    std::string name;

protected: 
    
    PerceptionActionInterface::PAI& pai;    
    const PerceptionActionInterface::ActionType& actionType;

public: 

    PetActionSchema(PerceptionActionInterface::PAI& pai, const PerceptionActionInterface::ActionType& actionType); 
    virtual ~PetActionSchema(); 

    const std::string& getName() const;
    bool isPetAction() const;
    combo::vertex execute(const std::vector<combo::vertex>& arguments) const throw (LADSUtil::RuntimeException, LADSUtil::InvalidParamException, std::bad_exception);
};

}  
 
#endif /*PET_ATION_SCHEMA_H_*/
