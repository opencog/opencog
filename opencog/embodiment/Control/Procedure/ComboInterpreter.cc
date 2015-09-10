/*
 * opencog/embodiment/Control/Procedure/ComboInterpreter.cc
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

#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/lazy_random_selector.h>
#include <opencog/util/lazy_normal_selector.h>

#include "ComboInterpreter.h"
#include <opencog/embodiment/Control/MessagingSystem/NetworkElement.h>
#include <opencog/embodiment/Control/MessagingSystem/StringMessage.h>
#include <boost/lexical_cast.hpp>

namespace opencog { namespace Procedure {

using namespace messaging;
using namespace boost;
using namespace std;

using world::PAIWorldWrapper;

ComboInterpreter::ComboInterpreter(PAI& p) : _ww(new PAIWorldWrapper(p)), _next(0)
{
}

ComboInterpreter::~ComboInterpreter()
{
}

void ComboInterpreter::run(messaging::NetworkElement *ne)
{
    if (_vec.empty())
        return;

    std::set<RunningProcedureId> done;

#if 0 // According to cpu time profiling, lazy_selector is taking too much time.
    lazy_selector* sel;
    if (!config().get_bool("AUTOMATED_SYSTEM_TESTS")) {
        //loop in random order until we find a running procedure that's ready
        //along the way, get rid of done procedures
        sel = new lazy_random_selector(_vec.size());
    } else {
        sel = new lazy_normal_selector(_vec.size());
    }

    while (!sel->empty()) {

        Vec::iterator it = _vec.begin() + (*sel)();
        RunningComboProcedure& rp = (*it)->second;

        if (rp.isReady()) {
            logger().debug("Running procedure id '%d'.",  ((RunningProcedureId&) (*it)->first).getId());
            rp.cycle();

        } else if (rp.isFinished()) {
            logger().debug("Done procedure id '%d'.", ((RunningProcedureId&) (*it)->first).getId());
            done.insert((*it)->first);
        }
    }
    delete sel;
#else
    
    for (Vec::iterator it = _vec.begin(); it != _vec.end(); ++it) {
        RunningComboProcedure& rp = (*it)->second;
        if (rp.isReady()) {
            logger().debug("Running procedure id '%d'.",  ((RunningProcedureId&) (*it)->first).getId());
            rp.cycle();

        } else if (rp.isFinished()) {
            logger().debug("Done procedure id '%d'.", ((RunningProcedureId&) (*it)->first).getId());
            done.insert((*it)->first);
        } /*else if (rp.isFailed()) {
            _failed.insert((*it)->first);
        }*/
    }
#endif

    Vec::iterator last = _vec.end();
    if (!done.empty()) {
        last = std::partition(_vec.begin(), _vec.end(), DonePred(done));
    }

    for (Vec::iterator it = last;it != _vec.end();++it) {
        RunningComboProcedure& rp = (*it)->second;
        if (!rp.isFinished()) {
            // Should be finished here. If not, record failure for querying and
            // send it to combo shell, if any
            _failed.insert((*it)->first);

            logger().error("Not finished '%d' - adding to failed list",  ((RunningProcedureId&) (*it)->first).getId());
            if (ne) {
                StringMessage msg(ne->getID(),
                                  config().get("COMBO_SHELL_ID"),
                                  "action_failure");
                ne->sendMessage(msg);
            }

        } else if (rp.getResult() != combo::id::null_vertex) { //stopped?
            if (!ne) {
                if (rp.isFailed()) {

                    _failed.insert((*it)->first); //record failures for querying
                } else {
                    _resultMap.insert(make_pair((*it)->first, rp.getResult()));
                }

            } else { //do the same and also send a message recording the result

                if (rp.isFailed()) {
                    _failed.insert((*it)->first);
                    StringMessage msg(ne->getID(),
                                      config().get("COMBO_SHELL_ID"),
                                      "action_failure");
                    ne->sendMessage(msg);

                } else {
                    _resultMap.insert(make_pair((*it)->first, rp.getResult()));
                    stringstream ss;
                    ss << rp.getResult();
                    StringMessage msg(ne->getID(),
                                      config().get("COMBO_SHELL_ID"),
                                      ss.str());
                    ne->sendMessage(msg);
                }
            }
        }
        _map.erase(*it);
    }
    _vec.erase(last, _vec.end());
}

//add a procedure to be run by the interpreter
RunningProcedureId ComboInterpreter::runProcedure(const combo::combo_tree& tr, const std::vector<combo::vertex>& arguments)
{
    RunningProcedureId id(++_next, COMBO);
    _vec.push_back(_map.insert(make_pair(id, RunningComboProcedure(*_ww, tr, arguments))).first);
    return id;
}

bool ComboInterpreter::isFinished(RunningProcedureId id) const
{
    Map::const_iterator it = _map.find(id);
    return (it == _map.cend() || it->second.isFinished());
}

// Note: this will return false if the stopProcedure() method was previously called for this same procedure id,
// even if the procedure execution has failed before
bool ComboInterpreter::isFailed(RunningProcedureId id) const
{
    if (_failed.find(id) != _failed.end()) {
        return true;
    }
    Map::const_iterator it = _map.find(id);

//    logger().warn("_map!end '%s', finished '%s', failed '%s'.",
//                    (it!=_map.end())?"true":"false", it->second.isFinished()?"true":"false", it->second.isFailed()?"true":"false");
    return (it != _map.cend() && it->second.isFinished() && it->second.isFailed());
}

// Get the result of the procedure with the given id
// Can be called only if the following conditions are true:
// - procedure execution is finished (checked by isFinished() method)
// - procedure execution has not failed (checked by isFailed() method)
// - procedure execution was not stopped (by calling stopProcedure() method)
combo::vertex ComboInterpreter::getResult(RunningProcedureId id)
{
    OC_ASSERT(isFinished(id), "ComboInterpreter - Procedure '%d' not finished.", id.getId());
    OC_ASSERT(!isFailed(id), "ComboInterpreter - Procedure '%d' failed.", id.getId());

    ResultMap::iterator it = _resultMap.find(id);

    if (it == _resultMap.end()) {
        Map::iterator mi = _map.find(id);
        OC_ASSERT(mi != _map.end(), "ComboInterpreter - Unable to find procedure '%d' in _map.", id.getId());
        return mi->second.getResult();
    }
    return it->second;
}

// makes the procedure with the given id to stop and remove it from the interpreter
void ComboInterpreter::stopProcedure(RunningProcedureId id)
{
    Map::iterator it = _map.find(id);
    if (it != _map.end()) {
        it->second.stop(); //stop in the middle
    } else {
        Set::iterator failedIt = _failed.find(id);
        if (failedIt != _failed.end()) {
            _failed.erase(failedIt);
        }
        ResultMap::iterator result_it = _resultMap.find(id);
        if (result_it != _resultMap.end()) {
            _resultMap.erase(result_it);
        }
    }
}

}} // ~namespace opencog::Procedure
