/*
 * opencog/spacetime/HandleTemporalPair.cc
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

#include "HandleTemporalPair.h"

#include <opencog/atoms/base/Atom.h>

using namespace opencog;

HandleTemporalPair::HandleTemporalPair()
{
}

HandleTemporalPair::HandleTemporalPair(Handle handle, Temporal* time)
{
    this->handle = handle;
    this->time = time;
}

HandleTemporalPair::~HandleTemporalPair()
{
}

Handle HandleTemporalPair::getHandle() const
{
    return handle;
}

Temporal* HandleTemporalPair::getTemporal() const
{
    return time;
}

std::string HandleTemporalPair::toString() const
{
    std::string  answer;
    answer += "(" + Handle(handle)->to_short_string() + "," + time->toString() + ")";
    return answer;
}

HandleTemporalPair HandleTemporalPair::clone()
{
    return HandleTemporalPair(handle, time);
}
