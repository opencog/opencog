/*
 * opencog/atomspace/TruthValue.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Welter Silva <welter@vettalabs.com>
 *            Guilherme Lamacie
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

#include <typeinfo>

#include <stdio.h>
#include <stdlib.h>

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/CountTruthValue.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include <opencog/atomspace/NullTruthValue.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/atomspace/TruthValue.h>
#include <opencog/util/platform.h>

//#define DPRINTF printf
#define DPRINTF(...)

using namespace opencog;

const strength_t MAX_TRUTH  = 1.0f;

TruthValuePtr TruthValue::NULL_TV()
{
    static TruthValuePtr instance(std::make_shared<NullTruthValue>());
    return instance;
}

TruthValuePtr TruthValue::DEFAULT_TV()
{
    static TruthValuePtr instance(std::make_shared<SimpleTruthValue>(MAX_TRUTH, 0.0));
    return instance;
}

TruthValuePtr TruthValue::TRUE_TV()
{
    static TruthValuePtr instance(std::make_shared<SimpleTruthValue>(MAX_TRUTH, 1.0e35));
    return instance;
}

TruthValuePtr TruthValue::FALSE_TV()
{
    static TruthValuePtr instance(std::make_shared<SimpleTruthValue>(0.0f, 1.0e35));
    return instance;
}

TruthValuePtr TruthValue::TRIVIAL_TV()
{
    static TruthValuePtr instance(std::make_shared<SimpleTruthValue>(0.0, 0.0));
    return instance;
}

bool TruthValue::isNullTv() const
{
    return false;
}

bool TruthValue::isDefaultTV() const
{
    TruthValuePtr dtv = DEFAULT_TV();
    if (dtv.get() == this) return true;
    if (getType() == dtv->getType() and
        getMean() == dtv->getMean() and
        getCount() == dtv->getCount())
    {
        return true;
    }
    return false;
}
