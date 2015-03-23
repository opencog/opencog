/*
 * opencog/embodiment/Control/Procedure/ComboInterpreter.h
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

#ifndef _COMBO_INTERPRETER_H
#define _COMBO_INTERPRETER_H

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/combo/variable_unifier.h>
#include <opencog/embodiment/Control/PerceptionActionInterface/PAI.h>
#include <opencog/embodiment/WorldWrapper/PAIWorldWrapper.h>
#include "RunningProcedureId.h"
#include "RunningComboProcedure.h"
#include <vector>
#include <boost/noncopyable.hpp>
#include <opencog/server/CogServer.h>
#include <opencog/embodiment/Control/MessagingSystem/NetworkElement.h>

namespace opencog { namespace Procedure {

using namespace pai;
using world::WorldWrapperBase;

typedef std::map<RunningProcedureId, RunningComboProcedure> Map;
typedef std::vector<Map::iterator> Vec;

//
class DonePred : public std::unary_function<Vec::iterator, bool>
{
private:
    std::set<RunningProcedureId> _set;

public:

    explicit DonePred(const std::set<RunningProcedureId>& s) : _set(s) {}

    // return true if the element is not in the set and false otherwise.
    // This will be used to split the predicates that are not finished yet
    // from the ones that already finished and must be removed from the
    // running procedure list
    bool operator()(Map::iterator& elem)  {
        return _set.find((*elem).first) == _set.end();
    }

}; // DoneSet

//
class ComboInterpreter : public boost::noncopyable
{

public:
    ComboInterpreter(PAI& p);
    virtual ~ComboInterpreter();

    //run executes a single action plan of some procedure (if any are ready)
    void run(messaging::NetworkElement *ne);

    //add a procedure to be run by the interpreter
    RunningProcedureId runProcedure(const combo::combo_tree& tr, const std::vector<combo::vertex>& arguments);

    bool isFinished(RunningProcedureId id) const;

    // Note: this will return false if the stopProcedure() method was previously called for this same procedure id,
    // even if the procedure execution has failed before
    bool isFailed(RunningProcedureId id) const;

    // Get the result of the procedure with the given id
    // Can be called only if the following conditions are true:
    // - procedure execution is finished (checked by isFinished() method)
    // - procedure execution has not failed (checked by isFailed() method)
    // - procedure execution was not stopped (by calling stopProcedure() method)
    combo::vertex getResult(RunningProcedureId id);

    // makes the procedure with the given id to stop and remove it from the interpreter
    void stopProcedure(RunningProcedureId id);

protected:
    typedef std::set<RunningProcedureId> Set;
    typedef std::map<RunningProcedureId, combo::vertex> ResultMap;

//    WorldWrapper::PAIWorldWrapper _ww;
    WorldWrapperBase * _ww;
    Map _map;
    Vec _vec;
    Set _failed;
    ResultMap _resultMap;

    unsigned long _next;

}; // ComboInterpreter


}} // ~namespace opencog::Procedure

#endif
