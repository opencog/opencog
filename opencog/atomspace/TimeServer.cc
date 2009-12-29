/*
 * opencog/atomspace/TimeServer.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
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

#include "TimeServer.h"

#include <opencog/util/Logger.h>

using namespace opencog;

int TimeServer::timeServerEntries = 0;
// USED TO SEEK MEMORY LEAK
//std::set<Temporal> TimeServer::temporalSet;

void TimeServer::init()
{
    table = new TemporalTable();
    latestTimestamp = 0;
}

TimeServer::TimeServer()
{
    init();
}

TimeServer::~TimeServer()
{
    delete table;
}

void TimeServer::add(Handle h, const Temporal& t)
{
    // USED TO SEEK MEMORY LEAK
    //++timeServerEntries;
    //cout << "Total timeServerEntries: " << timeServerEntries << endl;

    //if(temporalSet.find(t) == temporalSet.end()){
    //   temporalSet.insert(t);
    //   cout << "Total unique entrys: " << temporalSet.size() << endl;
    //}

    table->add(h, t);
    if (t.getUpperBound() > latestTimestamp) {
        latestTimestamp = t.getUpperBound();
    }
}

bool TimeServer::remove(Handle h, const Temporal& t, TemporalTable::TemporalRelationship criterion)
{
    return table->remove(h, t, criterion);
}

unsigned long TimeServer::getLatestTimestamp() const
{
    return latestTimestamp;
}

TimeServer& TimeServer::operator=(const TimeServer& other)
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "TimeServer - Cannot copy an object of this class");
}

TimeServer::TimeServer(const TimeServer& other) 
{
    throw opencog::RuntimeException(TRACE_INFO, 
            "TimeServer - Cannot copy an object of this class");
}

void TimeServer::clear()
{
    delete table;
    init();
}

