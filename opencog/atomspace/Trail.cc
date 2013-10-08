/*
 * opencog/atomspace/Trail.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Guilherme Lamacie
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

#include "Trail.h"

#include <stdio.h>

#include <opencog/atomspace/Atom.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/types.h>
#include <opencog/util/Logger.h>

#define DEFAULT_INITIAL_TRAIL_SIZE 0
#define DEFAULT_MAX_TRAIL_SIZE 1000

using namespace opencog;

void Trail::init(int initialSize, int max) throw (InvalidParamException, std::bad_exception)
{
    if (max < initialSize) {
        throw InvalidParamException(TRACE_INFO,
                                    "Trail - maxSize: %d < initialSize: %d.", max, initialSize);
    }
    if (initialSize != 0) {
        trail = new std::deque<Handle>(initialSize);
    } else {
        trail = NULL;
    }
    maxSize = max;
}

Trail::Trail() throw (InvalidParamException, std::bad_exception)
{
    init(DEFAULT_INITIAL_TRAIL_SIZE, DEFAULT_MAX_TRAIL_SIZE);
}

Trail::Trail(int initialSize) throw (InvalidParamException, std::bad_exception)
{
    init(initialSize, DEFAULT_MAX_TRAIL_SIZE);
}

Trail::Trail(int initialSize, int max) throw (InvalidParamException, std::bad_exception)
{
    init(initialSize, max);
}

Trail::~Trail()
{
    if (trail != NULL) delete trail;
}

bool Trail::isInTrail(Handle link)
{
    if (trail == NULL) return false;
    for (size_t i = 0; i < trail->size(); i++) {
        if (link == (*trail)[i])
            return(true);
    }
    return false;
}

size_t Trail::getSize()
{
    if (trail == NULL) return 0;
    return trail->size();
}

void Trail::insert(Handle link, bool fresh)
{
    if (trail == NULL) init(1, maxSize);

    if (fresh) {
        if (isInTrail(link)) return;
    }

    trail->push_back(link);

    if (maxSize < (int) trail->size()) {
        trail->pop_front();
    }

}

Handle Trail::getElement(int index) throw (IndexErrorException, std::bad_exception)
{

    if (trail == NULL) {
        throw IndexErrorException(TRACE_INFO,
                                  "Trail - invalid out of bounds (%d > %d).",
                                  index, 0);
    }

    if (index > (int)trail->size()) {
        throw IndexErrorException(TRACE_INFO,
                                  "Trail - invalid out of bounds (%d > %d).",
                                  index, trail->size());
    }
    return (*trail)[index];
}

void Trail::append(Trail* t)
{

    if (!t->getSize()) {
        return;
    }

    if (trail == NULL) init(1, maxSize);

    for (unsigned int i = 0; i < t->getSize(); i++) {
        Handle link = t->getElement(i);
        insert(link);
    }
}

void Trail::print()
{

    if (trail == NULL) {
        logger().debug("trailSize %d\n", 0);

    } else {
        logger().debug("trailSize %d\n", (int)trail->size());
        for (int i = 0; i < (int)trail->size(); i++) {
            logger().info("%d. %s\n", i, (*trail)[i]->toShortString().c_str());
        }
    }

    logger().info("endOfTrail\n\n");
}


void Trail::print(FILE* fp)
{

    if (trail == NULL) return;

    for (int i = 0; i < (int)trail->size(); i++) {
        fprintf(fp, "\t%d. %s\n", i, (*trail)[i]->toShortString().c_str());
    }
}

