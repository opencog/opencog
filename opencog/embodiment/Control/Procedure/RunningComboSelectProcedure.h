#ifndef _RUNNING_COMBO_SELECT_PROCEDURE_H
#define _RUNNING_COMBO_SELECT_PROCEDURE_H

#include "ComboProcedure.h"
#include "ComboInterpreter.h"

#include <ComboReduct/combo/vertex.h>
#include <ComboReduct/combo/variable_unifier.h>


namespace Procedure {

class RunningComboSelectProcedure {

    public:

        RunningComboSelectProcedure(ComboInterpreter& interpreter, 
                                    const ComboProcedure& f, 
                                    const ComboProcedure& s, 
                                    const std::vector<combo::vertex> args,
                                    combo::variable_unifier& vu = combo::variable_unifier::DEFAULT_VU());
        
        void cycle();

        bool isFinished() const;

        bool isFailed() const;

        combo::vertex getResult();

        combo::variable_unifier& getUnifierResult();

    private:
        ComboInterpreter& interpreter;

        ComboProcedure firstScript;
        ComboProcedure secondScript;

        std::vector<combo::vertex> arguments;

        // the results of 
        combo::variable_unifier unifier;
        combo::vertex result;

        bool firstScriptFinished; 
        bool firstScriptFailed;

        bool secondScriptFinished; 
        bool secondScriptFailed;

}; // class
}  // namespace

#endif
