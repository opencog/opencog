/*
 * opencog/embodiment/Control/Procedure/ProcedureInterpreter.cc
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

#include <opencog/util/mt19937ar.h>
#include <opencog/util/exceptions.h>

#include "ProcedureInterpreter.h"
#include "ComboProcedure.h"
#include "RunningProcedureId.h"
#include <opencog/embodiment/Control/MessagingSystem/NetworkElement.h>

namespace opencog { namespace Procedure {

using namespace boost;
using namespace pai;
using messaging::NetworkElement;

void ProcedureInterpreter::run(NetworkElement *ne)
{

    // If a separate Agent is used for comboInterpreter, just comment the
    // next line :
    comboInterpreter.run(ne);

    Set toBeRemoved;
    // Runs each pending RunningBuildInProcedure and checks status of any procedure.
    for (Map::iterator it = _map.begin(); it != _map.end(); ++it) {
        RunningBuiltInProcedure* rbp = get<RunningBuiltInProcedure>(&(it->second));
        if (rbp) {
            rbp->run();
            if (rbp->isFinished()) {
                toBeRemoved.insert(it->first);
                if (rbp->isFailed()) {
                    _failed.insert(it->first);
                } else {
                    _resultMap[it->first] = rbp->getResult();
                }
            }
        } else {
            RunningProcedureId rcpID = get<RunningProcedureId>(it->second);
            if (comboInterpreter.isFinished(rcpID)) {
                toBeRemoved.insert(it->first);
                if (comboInterpreter.isFailed(rcpID)) {
                    _failed.insert(it->first);
                } else {
                    _resultMap[it->first] = comboInterpreter.getResult(rcpID);
                }
            }
        }

    }
    for (Set::iterator it = toBeRemoved.begin(); it != toBeRemoved.end(); ++it) {
        _map.erase(*it);
    }
}

ProcedureInterpreter::ProcedureInterpreter(PAI& p)
    : _pai(p), comboInterpreter(_pai), _next(0)
{
    // Initialize the random generator
    unsigned long rand_seed;
    if (config().get_bool("AUTOMATED_SYSTEM_TESTS")) {
        rand_seed = 0;
    } else {
        rand_seed = time(NULL);
    }
    randGen().seed(rand_seed);
    logger().info("Created random number generator for ComboInterpreter with seed %lu", rand_seed);
}

ProcedureInterpreter::~ProcedureInterpreter()
{
}

RunningProcedureID ProcedureInterpreter::runProcedure(
        const GeneralProcedure& p, const std::vector<combo::vertex>& arguments)
{
    logger().info("ProcedureInterpreter - runProcedure(%s)", p.getName().c_str());

    if (p.getType() == COMBO) {
        logger().debug("ProcedureInterpreter - Running a combo procedure.");

        RunningProcedureId rcpID = comboInterpreter.runProcedure(((const ComboProcedure&) p).getComboTree(), arguments);
        _map.insert(std::make_pair(++_next, rcpID));
    } else if (p.getType() == BUILT_IN) {
        logger().debug("ProcedureInterpreter - Running a builtin procedure.");
        RunningBuiltInProcedure rbp = RunningBuiltInProcedure(_pai, (const BuiltInProcedure&) p, arguments);
        // For now, runs built-in procedure immediately, since they are atomic
        // and caller may want to check for failure or get its result synchronously.
        rbp.run();
        _map.insert(std::make_pair(++_next, rbp));
    } else {
        OC_ASSERT(false, "ProcedureInterpreter -  unknown procedure type");
    }
    return _next;
}

RunningProcedureID ProcedureInterpreter::runProcedure(const GeneralProcedure& p, const std::vector<combo::vertex>& arguments, combo::variable_unifier& vu)
{
    logger().info(
                 "ProcedureInterpreter - runProcedure(%s)", p.getName().c_str());

    if (p.getType() == COMBO) {
        logger().debug(
                     "ProcedureInterpreter - Running a combo procedure.");
        RunningProcedureId rcpID = comboInterpreter.runProcedure(((const ComboProcedure&) p).getComboTree(), arguments);
        _map.insert(std::make_pair(++_next, rcpID));

    } else {
        OC_ASSERT(false, "ProcedureInterpreter - Only combo procedures accept variable unifier parameters.");
    }
    return _next;
}

bool ProcedureInterpreter::isFinished(RunningProcedureID id) const
{
    logger().fine("ProcedureInterpreter - isFinished(%lu).", id);
    bool result = true;
    Map::const_iterator it = _map.find(id);
    if (it != _map.end()) {
        RunningProcedure rp = it->second;
        RunningProcedureId* rpId;
        RunningBuiltInProcedure* rbp;
        if ((rbp = boost::get<RunningBuiltInProcedure>(&rp))) {
            result = rbp->isFinished();
        } else if ((rpId = boost::get<RunningProcedureId>(&rp))) {
            result = comboInterpreter.isFinished(*rpId);
        }
    }
    logger().debug("ProcedureInterpreter - isFinished(%lu)? Result: %d.",
                 id, result);
    return result;
}

bool ProcedureInterpreter::isFailed(RunningProcedureID id) const
{
    logger().fine("ProcedureInterpreter - isFailed(%lu).", id);
    bool result = false;
    Map::const_iterator it = _map.find(id);
    if (it != _map.end()) {
        const RunningProcedureId* rpId;

        if ((rpId = boost::get<RunningProcedureId>(&(it->second)))) {
            result = comboInterpreter.isFailed(*rpId);

        } else {
            result = boost::get<RunningBuiltInProcedure>(it->second).isFailed();
        }
    } else {
        result = (_failed.find(id) != _failed.end());
    }
    logger().debug("ProcedureInterpreter - isFailed(%lu)? Result: %d.",
                 id, result);
    return result;
}

combo::vertex ProcedureInterpreter::getResult(RunningProcedureID id)
{
    logger().fine("ProcedureInterpreter - getResult(%lu).", id);
    OC_ASSERT(isFinished(id), "ProcedureInterpreter - Procedure '%d' not finished.", id);
    OC_ASSERT(!isFailed(id), "ProcedureInterpreter - Procedure '%d' failed.", id);

    combo::vertex result = combo::id::action_success;
    Map::const_iterator it = _map.find(id);

    if (it != _map.end()) {
        RunningBuiltInProcedure* rbp;
        RunningProcedureId* rpId;
        RunningProcedure rp = it->second;

        if ((rbp = boost::get<RunningBuiltInProcedure>(&rp))) {
            result = rbp->getResult();

        } else if ((rpId = boost::get<RunningProcedureId>(&rp))) {
            result = comboInterpreter.getResult(*rpId);
        }

    } else {
        ResultMap::iterator it = _resultMap.find(id);
        OC_ASSERT(it != _resultMap.end(),
                         "ProcedureInterpreter - Cannot find result for procedure '%d'.", id);
        result = it->second;
    }
    return result;
}

combo::variable_unifier& ProcedureInterpreter::getUnifierResult(RunningProcedureID id)
{
    OC_ASSERT(isFinished(id), "ProcedureInterpreter - Procedure '%d' not finished.", id);
    OC_ASSERT(!isFailed(id), "ProcedureInterpreter - Procedure '%d' failed.", id);
    UnifierResultMap::iterator it = _unifierResultMap.find(id);
    OC_ASSERT(it != _unifierResultMap.end(),
                     "ProcedureInterpreter - Cannot find unifier result for procedure '%d'.", id);
    return it->second;
}

void ProcedureInterpreter::stopProcedure(RunningProcedureID id)
{
    logger().fine("ProcedureInterpreter - stopProcedure(%lu).", id);
    Map::iterator it = _map.find(id);
    if (it != _map.end()) {
        RunningProcedureId* rpId = boost::get<RunningProcedureId>(&(it->second));
        comboInterpreter.stopProcedure(*rpId);
        _map.erase(it);
    }
    Set::iterator failed_it = _failed.find(id);
    if (failed_it != _failed.end()) {
        _failed.erase(failed_it);
    }
    ResultMap::iterator result_it = _resultMap.find(id);
    if (result_it != _resultMap.end()) {
        _resultMap.erase(result_it);
    }
}

const ComboInterpreter& ProcedureInterpreter::getComboInterpreter() const
{
    return comboInterpreter;
}

} // ~namespace Procedure
} // ~namespace opencog


