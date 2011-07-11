/*
 * opencog/atomspace/HandleSet.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Rodrigo Barra
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

#include "HandleSet.h"
#include "TLB.h"

using namespace opencog;

HandleSet::HandleSet()
{
    handleSet = new InternalHandleSet();
}

HandleSet::HandleSet(InternalHandleSet *set)
{
    handleSet = set;
}

HandleSet::~HandleSet()
{
    delete(handleSet);
}

HandleSet *HandleSet::clone()
{
    return(new HandleSet(new InternalHandleSet(*handleSet)));
}

void HandleSet::add(Handle h)
{
    handleSet->insert(h);
}

void HandleSet::add(HandleSet *hs)
{
    handleSet->insert(hs->handleSet->begin(), hs->handleSet->end());
}

bool HandleSet::contains(Handle h) const
{
    InternalIterator it = handleSet->find(h);

    return(it != handleSet->end());
}


void HandleSet::remove(Handle h) throw (RuntimeException)
{
    InternalIterator it = handleSet->find(h);

    if (it != handleSet->end()) {
        handleSet->erase(it);
    } else {
        throw RuntimeException(TRACE_INFO, "Could not remove inexistent Handle");
    }
}

int HandleSet::getSize()
{
    return(handleSet->size());
}

HandleSetIterator *HandleSet::keys()
{
    return new HandleSetIterator(this);
}

std::string HandleSet::toString()
{
    std::string answer;
    for (HandleSetIterator *it = keys(); it->hasNext();) {
        Handle key = it->next();
        /* append key */
        answer += TLB::getAtom(key)->toShortString();
        if (it->hasNext()) {
            answer += ",";
        }
    }
    return answer;
}


HandleSetIterator::HandleSetIterator(HandleSet *set)
{
    this->set = set;
    current = set->handleSet->begin();
}

bool HandleSetIterator::hasNext()
{
    return current != set->handleSet->end();
}

Handle HandleSetIterator::next() throw (IndexErrorException)
{
    if (!hasNext()) {
        throw IndexErrorException(TRACE_INFO, "HandleSetIterator out of bounds");
    }

    Handle ret = *current;

    ++current;

    return(ret);
}
