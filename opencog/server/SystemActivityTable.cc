/*
 * opencog/server/SystemActivityTable.cc
 *
 * Copyright (C) 2009 by OpenCog Foundation
 * Copyright (C) 2010-2011 OpenCog Foundation
 * All Rights Reserved
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

#include "SystemActivityTable.h"

#include <opencog/util/Logger.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/platform.h>
#include <opencog/atomspace/AtomSpace.h>
#include <opencog/server/CogServer.h>

using namespace opencog;

SystemActivityTable::SystemActivityTable() : _maxAgentActivityTableSeqSize(100)
{
    logger().debug("[SystemActivityTable] constructor");
}

SystemActivityTable::~SystemActivityTable()
{
    logger().debug("[SystemActivityTable] enter destructor");
    _conn.disconnect();
    logger().debug("[SystemActivityTable] exit destructor");
}

void SystemActivityTable::init(CogServer *cogServer)
{
    logger().debug("[SystemActivityTable] init");
    _cogServer = cogServer;
    _conn = cogServer->getAtomSpace().removeAtomSignal(
            boost::bind(&SystemActivityTable::atomRemoved, this, _1));
}

void SystemActivityTable::setMaxAgentActivityTableSeqSize(size_t n)
{
    _maxAgentActivityTableSeqSize = n;

    for (AgentActivityTable::iterator it  = _agentActivityTable.begin();
                                      it != _agentActivityTable.end(); ++it) {
        ActivitySeq &seq = it->second;
        trimActivitySeq(seq, _maxAgentActivityTableSeqSize);
    }
}

void SystemActivityTable::trimActivitySeq(ActivitySeq &seq, size_t max)
{
    if (seq.size() <= max)
        return;
    for (size_t n = max; n < seq.size(); n++)
        delete seq[n];
    seq.resize(max);
}

void SystemActivityTable::atomRemoved(AtomPtr atom)
{
    Handle h = atom->getHandle();
    for (AgentActivityTable::iterator it  = _agentActivityTable.begin();
                                      it != _agentActivityTable.end(); ++it) {
        ActivitySeq &seq = it->second;
        for (size_t n = 0; n < seq.size(); n++) {
            Activity *a = seq[n];
            for (size_t i = 0; i < a->utilizedHandleSets.size(); i++) {
                a->utilizedHandleSets[i].erase(h);
            }
        }
    }
}

void SystemActivityTable::logActivity(AgentPtr agent, struct timeval &elapsedTime, 
                                      size_t memUsed, size_t atomsUsed)
{
    ActivitySeq& as = _agentActivityTable[agent];
    as.insert(as.begin(), 
        new Activity(_cogServer->getCycleCount(), elapsedTime, memUsed,
                 atomsUsed,
                 agent->getUtilizedHandleSets()));
    trimActivitySeq(as, _maxAgentActivityTableSeqSize);
}

void SystemActivityTable::clearActivity(AgentPtr agent)
{
    AgentActivityTable::iterator it = _agentActivityTable.find(agent);
    if (it == _agentActivityTable.end())
        return;
    ActivitySeq &seq = it->second;
    for (size_t n = 0; n < seq.size(); n++)
        delete seq[n];
    _agentActivityTable.erase(it);
}

void SystemActivityTable::clearActivity()
{
    for (AgentActivityTable::iterator it  = _agentActivityTable.begin();
                                      it != _agentActivityTable.end(); ++it) {
        ActivitySeq& seq = it->second;
        for (size_t n = 0; n < seq.size(); n++)
            delete seq[n];
    }
    _agentActivityTable.clear();
}

