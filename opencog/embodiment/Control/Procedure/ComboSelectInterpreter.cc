/*
 * opencog/embodiment/Control/Procedure/ComboSelectInterpreter.cc
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

#include "ComboSelectInterpreter.h"

#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>

namespace opencog { namespace Procedure {

ComboSelectInterpreter::ComboSelectInterpreter(pai::PAI& pai)
{
    this->comboInterpreter = new ComboInterpreter(pai);
    this->next = 0;
}

ComboSelectInterpreter::~ComboSelectInterpreter()
{
    delete this->comboInterpreter;
}

void ComboSelectInterpreter::run(messaging::NetworkElement* ne)
{
    if (runningProc.empty()) return;

    // select the head of the map, since the RunningId is the map's key
    idProcedureMap::iterator it = runningProc.begin();
    RunningComboSelectProcedure& rp = it->second;

    rp.cycle();
    logger().debug("RunningComboSelect - Terminei o cycle.");

    if (!rp.isFinished()) {

        logger().debug(
                     "RunningComboSelect - Procedure not finished. Marking it failed.");

        // failed -  should be finished
        failed.insert(it->first);

    } else if (rp.getResult() != combo::id::null_vertex) {
        logger().debug(
                     "RunningComboSelect - Procedure finished.");

        if (rp.isFailed()) {
            failed.insert(it->first);
        } else {
            result.insert(make_pair(it->first, rp.getResult()));
            unifier.insert(make_pair(it->first, rp.getUnifierResult()));
        }
    } else {
        std::stringstream ss;
        ss << rp.getResult();
        logger().debug(
                     "Third else - '%s'", ss.str().c_str());
    }
    runningProc.erase(it);
}

Procedure::RunningProcedureId ComboSelectInterpreter::runProcedure(
    const ComboProcedure& f,
    const ComboProcedure& s,
    const std::vector<combo::vertex> arguments)
{
    RunningProcedureId id(++next, COMBO_SELECT);
    runningProc.insert(std::make_pair(id, RunningComboSelectProcedure(*comboInterpreter, f, s, arguments)));
    return id;
}


Procedure::RunningProcedureId ComboSelectInterpreter::runProcedure(
    const ComboProcedure& f,
    const ComboProcedure& s,
    const std::vector<combo::vertex> arguments,
    combo::variable_unifier& vu)
{

    RunningProcedureId id(++next, COMBO_SELECT);
    runningProc.insert(std::make_pair(id, RunningComboSelectProcedure(*comboInterpreter, f, s, arguments, vu)));
    return id;
}

bool ComboSelectInterpreter::isFinished(Procedure::RunningProcedureId id)
{
    idProcedureMap::const_iterator it = runningProc.find(id);
    return (it == runningProc.end() || it->second.isFinished());
}

bool ComboSelectInterpreter::isFailed(Procedure::RunningProcedureId id)
{
    if (failed.find(id) != failed.end()) {
        return true;
    }

    idProcedureMap::const_iterator it = runningProc.find(id);
    return (it != runningProc.end() && it->second.isFinished() && it->second.isFailed());
}

combo::vertex ComboSelectInterpreter::getResult(RunningProcedureId id)
{
    OC_ASSERT(isFinished(id), "ComboSelectInterpreter - Procedure '%lu' not finished.", id.getId());
    OC_ASSERT(!isFailed(id),  "ComboSelectInterpreter - Procedure '%lu' failed.", id.getId());

    idVertexMap::iterator it = result.find(id);

    if (it == result.end()) {
        idProcedureMap::iterator runningProcIt = runningProc.find(id);
        if (runningProcIt == runningProc.end()) {
            OC_ASSERT(false, "ERROR.");
        }

        OC_ASSERT(runningProcIt->second.isFinished(), "ComboSelectInterpreter - Procedure '%lu' not finished.", id.getId());
        return runningProcIt->second.getResult();
    }
    return it->second;
}

combo::variable_unifier& ComboSelectInterpreter::getUnifierResult(RunningProcedureId id)
{
    OC_ASSERT(isFinished(id), "ComboSelectInterpreter - Procedure '%lu' not finished.", id.getId());
    OC_ASSERT(!isFailed(id),  "ComboSelectInterpreter - Procedure '%lu' failed.", id.getId());

    idUnifierMap::iterator it = unifier.find(id);

    if (it == unifier.end()) {
        idProcedureMap::iterator runningProcIt = runningProc.find(id);
        if (runningProcIt == runningProc.end()) {
            //error
        }
        return runningProcIt->second.getUnifierResult();
    }
    return it->second;
}

void ComboSelectInterpreter::stopProcedure(RunningProcedureId id)
{

    idProcedureMap::iterator it = runningProc.find(id);
    if (it != runningProc.end()) {
//        it->second.stop();
    }

    std::set<RunningProcedureId>::iterator failedIt = failed.find(id);
    if (failedIt != failed.end()) {
        failed.erase(failedIt);
    }

    idVertexMap::iterator resultIt = result.find(id);
    if (resultIt != result.end()) {
        result.erase(resultIt);
    }

    idUnifierMap::iterator unifierIt = unifier.find(id);
    if (unifierIt != unifier.end()) {
        unifier.erase(unifierIt);
    }

}

}} // ~namespace opencog::Procedure
