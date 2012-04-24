/*
 * opencog/embodiment/Control/Procedure/ComboSelectInterpreter.h
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

#ifndef _COMBO_SELECT_INTERPRETER_H
#define _COMBO_SELECT_INTERPRETER_H

#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>
#include "ComboProcedure.h"
#include "ComboInterpreter.h"
#include "RunningProcedureId.h"
#include "RunningComboSelectProcedure.h"

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/combo/variable_unifier.h>

namespace opencog { namespace Procedure {

class ComboSelectInterpreter
{

public:

    ComboSelectInterpreter(pai::PAI& pai);
    ~ComboSelectInterpreter();

    // from idle task
    void run(messaging::NetworkElement *ne);

    /**
     * add a ComboSelect procedure to be run by the interpreter.
     *
     * @param f The first combo script
     * @param s The second combo script
     * @param arguments The overall procedure arguments. Currently this will
     *        empty most of the time.
     */
    Procedure::RunningProcedureId runProcedure(const ComboProcedure& f, const ComboProcedure& s,
            const std::vector<combo::vertex> arguments);

    /**
     * add a ComboSelect procedure to be run by the interpreter with a
     * variable unifier.
     *
     * @param f The first combo script
     * @param s The second combo script
     * @param arguments The overall procedure arguments. Currently this will
     *        empty most of the time.
     * @param vu The variable unifier
     */
    Procedure::RunningProcedureId runProcedure(const ComboProcedure& f, const ComboProcedure& s,
            const std::vector<combo::vertex> arguments,
            combo::variable_unifier& vu);

    //
    bool isFinished(Procedure::RunningProcedureId id);

    //
    bool isFailed(Procedure::RunningProcedureId id);

    // Get the result of the procedure with the given id
    // Can be called only if the following conditions are true:
    // - procedure execution is finished (checked by isFinished() method)
    // - procedure execution has not failed (checked by isFailed() method)
    // - procedure execution was not stopped (by calling stopProcedure() method)
    combo::vertex getResult(RunningProcedureId id);

    // Get the result of the variable unification carried within the procedure
    // execution.
    // Can be called only if the following conditions are true:
    // - procedure execution is finished (checked by isFinished() method)
    // - procedure execution has not failed (checked by isFailed() method)
    // - procedure execution was not stopped (by calling stopProcedure() method)
    combo::variable_unifier& getUnifierResult(RunningProcedureId id);

    // stop the running procedure and remove it from result, failed and
    // unifer maps, if present there.
    void stopProcedure(RunningProcedureId id);

private:

    typedef std::map<RunningProcedureId, RunningComboSelectProcedure> idProcedureMap;
    typedef std::map<RunningProcedureId, combo::variable_unifier> idUnifierMap;
    typedef std::map<RunningProcedureId, combo::vertex> idVertexMap;

    idProcedureMap runningProc;
    idUnifierMap unifier;
    idVertexMap result;

    std::set<RunningProcedureId> failed;

    unsigned long next;
    ComboInterpreter * comboInterpreter;

}; // class

} // ~namespace Procedure
} // ~namespace opencog
        
#endif
