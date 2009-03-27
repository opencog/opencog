#ifndef _PROCEDURE_INTERPRETER_H
#define _PROCEDURE_INTERPRETER_H

#include "ComboInterpreter.h"
#include "ComboSelectInterpreter.h"
#include "RunningBuiltInProcedure.h"

#include <boost/noncopyable.hpp>

namespace Procedure {

typedef unsigned long int RunningProcedureID;
typedef boost::variant<RunningProcedureId, RunningBuiltInProcedure> RunningProcedure;

class ProcedureInterpreter : public boost::noncopyable {

public: 
    ProcedureInterpreter(PerceptionActionInterface::PAI& p);

    ~ProcedureInterpreter();

    // call ComboInterpreter::run() method and execute pending running BuiltIn procedures.
    void run(MessagingSystem::NetworkElement *ne);

    // add a procedure to be run by the interpreter
    RunningProcedureID runProcedure(const GeneralProcedure& p, const std::vector<combo::vertex>& arguments);
    
    // add a procedure to be run by the interpreter
    RunningProcedureID runProcedure(const GeneralProcedure& p, const std::vector<combo::vertex>& arguments, combo::variable_unifier& vu);
    
    bool isFinished(RunningProcedureID id) const;
    
    // Note: this will return false if the stopProcedure() method was previously called for this same procedure id, 
    // even if the procedure execution has failed before
    bool isFailed(RunningProcedureID id) const;
    
    // Get the result of the procedure with the given id
    // Can be called only if the following conditions are true:
    // - procedure execution is finished (checked by isFinished() method)
    // - procedure execution has not failed (checked by isFailed() method)
    // - procedure execution was not stopped (by calling stopProcedure() method) 
    combo::vertex getResult(RunningProcedureID id);
    
    // Get the result of the variable unification carried within the procedure
    // execution. THIS METHOD IS USED ONLY FOR COMBO PROCEDURES
    // Can be called only if the following conditions are true:
    // - procedure execution is finished (checked by isFinished() method)
    // - procedure execution has not failed (checked by isFailed() method)
    // - procedure execution was not stopped (by calling stopProcedure() method)     
    combo::variable_unifier& getUnifierResult(RunningProcedureID id);

    // makes the procedure with the given id to stop and remove it from the interpreter
    void stopProcedure(RunningProcedureID id);

    // return the combo interpreter object
    ComboInterpreter& getComboInterpreter() const;
    
    // return the combo select interpreter object
    ComboSelectInterpreter& getComboSelectInterpreter() const;
    
protected:
    typedef std::map<RunningProcedureID,RunningProcedure> Map;
    typedef std::vector<Map::iterator> Vec;
    typedef std::set<RunningProcedureID> Set;
    typedef std::map<RunningProcedureID,combo::vertex> ResultMap;
    typedef std::map<RunningProcedureID,combo::variable_unifier> UnifierResultMap;

    opencog::RandGen* rng;
    ComboInterpreter* comboInterpreter;
    ComboSelectInterpreter* comboSelectInterpreter;
    Map _map;
    Set _failed;
    ResultMap _resultMap;
    UnifierResultMap _unifierResultMap;
    
    PerceptionActionInterface::PAI* _pai;

    RunningProcedureID _next;
};

} //~namespace Procedure

#endif
