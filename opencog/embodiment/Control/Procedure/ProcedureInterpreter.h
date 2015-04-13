/*
 * opencog/embodiment/Control/Procedure/ProcedureInterpreter.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Novamente team
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _PROCEDURE_INTERPRETER_H
#define _PROCEDURE_INTERPRETER_H

#include "ComboInterpreter.h"
#include "RunningBuiltInProcedure.h"

#include <boost/noncopyable.hpp>

namespace opencog { namespace Procedure {

// BE CAREFUL!!!!!! this is RunningProcedureID ending with ID (all
// upper case), different from what is below in the definition of
// RunningProcedure that uses RunningProcedureId ending with Id
// (capitalized)
typedef unsigned long int RunningProcedureID;
typedef boost::variant<RunningProcedureId, RunningBuiltInProcedure> RunningProcedure;

class ProcedureInterpreter : public boost::noncopyable
{

public:
    ProcedureInterpreter(pai::PAI& p);

    ~ProcedureInterpreter();

    //! call ComboInterpreter::run() method and execute pending running BuiltIn procedures.
    void run(messaging::NetworkElement *ne);

    /** Add a procedure to be run by the interpreter
     * @param p The procedure to run.
     * @param arguments The arguments to the procedure.
     * @return The procedure ID
     */
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
    // XXX this method is not used anywhere
    const ComboInterpreter& getComboInterpreter() const;

protected:
    typedef std::map<RunningProcedureID, RunningProcedure> Map;
    typedef std::vector<Map::iterator> Vec;
    typedef std::set<RunningProcedureID> Set;
    typedef std::map<RunningProcedureID, combo::vertex> ResultMap;
    typedef std::map<RunningProcedureID, combo::variable_unifier> UnifierResultMap;

    pai::PAI& _pai;
    ComboInterpreter comboInterpreter;
    Map _map;
    Set _failed;
    ResultMap _resultMap;
    UnifierResultMap _unifierResultMap;

    RunningProcedureID _next;
};

} // ~namespace Procedure
} // ~namespace opencog

#endif
