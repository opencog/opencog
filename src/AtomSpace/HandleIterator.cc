/*
 * src/AtomSpace/HandleIterator.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Thiago Maia <thiago@vettatech.com>
 *            Andre Senna <senna@vettalabs.com>
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

#include <platform.h>
#include "HandleIterator.h"
#include "AtomTable.h"
#include "classes.h"
#include "ClassServer.h"
#include "TLB.h"
#include "AtomSpaceDefinitions.h"

HandleIterator::HandleIterator( AtomTable *t, Type type, bool subclass, VersionHandle vh) {
    init(t, type, subclass, vh);
}

void HandleIterator::init( AtomTable *t, Type type, bool subclass, VersionHandle vh) {

	table = t;

    desiredType = type;
    desiredTypeSubclass = subclass;
    desiredVersionHandle = vh;
    
    if (subclass) {
        // current handle and type are set to be the first element in the
        // first index that matches subclass criteria
        currentType = 0;
        while (currentType < ClassServer::getNumberOfClasses()) {
            if ((ClassServer::isAssignableFrom(desiredType, currentType)) &&
                !TLB::isInvalidHandle(table->getTypeIndexHead(currentType)))
            {
                break;
            } else {
                currentType++;
            }
        }
        currentHandle = currentType >= ClassServer::getNumberOfClasses() ? UNDEFINED_HANDLE : table->getTypeIndexHead(currentType);
    } else {
        // if no subclasses are allowed, the current handle is simply
        // the first element of the desired type index.
        currentHandle = table->getTypeIndexHead(type);
        currentType = type;
    }

    // the AtomTable will notify this iterator when some atom is removed to
    // prevent this iterator from iterating through a removed atom
    table->registerIterator(this);
}

HandleIterator::~HandleIterator() {
    table->unregisterIterator(this);
}

bool HandleIterator::hasNext() {
    return !TLB::isInvalidHandle(currentHandle);
}

Handle HandleIterator::next() {

    // keep current handle to return it
    Handle answer = currentHandle;

    // the iterator goes to the next position of the current list.
    currentHandle = TLB::getAtom(currentHandle)->next(TYPE_INDEX);

    // if the list finishes, it's necessary to move to the next index
    // that matches subclass criteria
    if (TLB::isInvalidHandle(currentHandle))
    {
        if (desiredTypeSubclass) {
            currentType++;
            while (currentType < ClassServer::getNumberOfClasses()) {
                if ((ClassServer::isAssignableFrom(desiredType, currentType)) &&
                    !TLB::isInvalidHandle(table->getTypeIndexHead(currentType)))
                {
                    break;
                } else {
                    currentType++;
                }
            }
            // currentHandle is the first element of the next index.
            currentHandle = currentType >= ClassServer::getNumberOfClasses() ? UNDEFINED_HANDLE : table->getTypeIndexHead(currentType);
        }
    }

    return answer;
}
