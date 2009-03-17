#ifndef _RUNNING_BUILTIN_PROCEDURE_H
#define _RUNNING_BUILTIN_PROCEDURE_H

#include "BuiltInProcedure.h"
#include "PAI.h"

namespace Procedure {

class RunningBuiltInProcedure {
    
    public: 
        RunningBuiltInProcedure(const PerceptionActionInterface::PAI& _pai, const BuiltInProcedure& _p, const std::vector<combo::vertex>& _arguments);
        ~RunningBuiltInProcedure();
        
        void run();
         
        bool isFinished() const;
        bool isFailed() const;
        combo::vertex getResult() const;

    protected:
    
        const PerceptionActionInterface::PAI& pai;
        const BuiltInProcedure& p;
        bool finished; 
        bool failed;
        combo::vertex result;
        std::vector<combo::vertex> arguments; 
};

} //~namespace Procedure

#endif
