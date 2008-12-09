/*
 * opencog/atomspace/HandleIterator.cc
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

#include "HandleIterator.h"

#include <opencog/util/platform.h>
#include <opencog/atomspace/AtomTable.h>

using namespace opencog;

HandleIterator::HandleIterator(AtomTable *t, Type type, bool subclass,
                               VersionHandle vh) :
	it(type, subclass)
{
    table = t;
    it = table->typeIndex.begin(type, subclass);
    table->registerIterator(this);
    desiredVersionHandle = vh;
}

HandleIterator::~HandleIterator()
{
    table->unregisterIterator(this);
}

bool HandleIterator::hasNext(void)
{
    return it != table->typeIndex.end();
}

Handle HandleIterator::next(void)
{
    Handle h = *it;
    it++;
    return h;
}
