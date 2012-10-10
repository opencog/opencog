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

#include "TruthValue.h"

#include <typeinfo>

#include <stdio.h>
#include <stdlib.h>

#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/CompositeTruthValue.h>
#include <opencog/atomspace/CountTruthValue.h>
#include <opencog/atomspace/IndefiniteTruthValue.h>
#include <opencog/atomspace/NullTruthValue.h>
#include <opencog/atomspace/SimpleTruthValue.h>
#include <opencog/util/platform.h>

//#define DPRINTF printf
#define DPRINTF(...)

using namespace opencog;

const TruthValue& TruthValue::NULL_TV()
{
    static TruthValue* instance = new NullTruthValue();
    return *instance;
}

const TruthValue& TruthValue::DEFAULT_TV()
{
    static TruthValue* instance = new SimpleTruthValue(0, 0);
    return *instance;
}

const TruthValue& TruthValue::TRUE_TV()
{
    static TruthValue* instance = new SimpleTruthValue(MAX_TRUTH, 1.0e35);
    return *instance;
}

const TruthValue& TruthValue::FALSE_TV()
{
    static TruthValue* instance = new SimpleTruthValue(0.0f, 1.0e35);
    return *instance;
}

const TruthValue& TruthValue::TRIVIAL_TV()
{
    static TruthValue* instance = new SimpleTruthValue(MAX_TRUTH, 0.0f);
    return *instance;
}

TruthValue& TruthValue::operator=(const TruthValue & rhs)
{
    return *this;
}

TruthValue* TruthValue::merge(const TruthValue& other) const
{
#if 1
    // TODO: Use the approach with dynamic cast below
    // if we're going to have subclasses of CompositeTruthValue.
    // For now, this approach using getType() is more efficient.
    if (other.getType() == COMPOSITE_TRUTH_VALUE) {
#else
    const CompositeTruthValue *otherCTv =
        dynamic_cast<const CompositeTruthValue *>(&other);
    if (otherCTv) {
#endif
        return other.merge(*this);
    } else if (other.getConfidence() > getConfidence()) {
        return other.clone();
    }
    return clone();
}

bool TruthValue::isNullTv() const
{
    return false;
}

bool TruthValue::isDefaultTV() const
{
    const TruthValue& dtv = TruthValue::DEFAULT_TV();
    if (this == &(dtv)) return true;
    if (getType() == dtv.getType() &&
        getMean() == dtv.getMean() &&
        getCount() == dtv.getCount()) {
        return true;
    }
    return false;
}

const char* TruthValue::typeToStr(TruthValueType t) throw (InvalidParamException)
{
    switch (t) {
    case SIMPLE_TRUTH_VALUE:
        return "SIMPLE_TRUTH_VALUE";
    case COUNT_TRUTH_VALUE:
        return "COUNT_TRUTH_VALUE";
    case INDEFINITE_TRUTH_VALUE:
        return "INDEFINITE_TRUTH_VALUE";
    case COMPOSITE_TRUTH_VALUE:
        return "COMPOSITE_TRUTH_VALUE";
    default:
        throw InvalidParamException(TRACE_INFO,
                                    "TruthValue - Invalid Truth Value type: '%d'.", t);
    }
}

TruthValueType TruthValue::strToType(const char* str) throw (InvalidParamException)
{
    DPRINTF("TruthValue::strToType(%s)\n", str);
    TruthValueType t = SIMPLE_TRUTH_VALUE;

    while (t != NUMBER_OF_TRUTH_VALUE_TYPES) {
        if (!strcmp(str, typeToStr(t))) {
            return t;
        }
        t = (TruthValueType)((int)t + 1);
    }

    throw InvalidParamException(TRACE_INFO,
                                "TruthValue - Invalid Truth Value type string: '%s'.", str);
}

TruthValue* TruthValue::factory(const char* fullTvStr)
{
    DPRINTF("TruthValue::factory(): fullTvStr = %s\n", fullTvStr);
    char typeStr[MAX_TRUTH_VALUE_NAME_LEN];
    sscanf(fullTvStr, "%s", typeStr);
    TruthValueType type = strToType(typeStr);
    return factory(type, fullTvStr + strlen(typeStr) + 1);
}

TruthValue* TruthValue::factory(TruthValueType type, const char* tvStr) throw (InvalidParamException)
{
    DPRINTF("TruthValue::factory(): type = %s tvStr = %s\n", TruthValue::typeToStr(type), tvStr);
    switch (type) {
    case SIMPLE_TRUTH_VALUE:
        return SimpleTruthValue::fromString(tvStr);
        break;
    case COUNT_TRUTH_VALUE:
        return CountTruthValue::fromString(tvStr);
        break;
    case INDEFINITE_TRUTH_VALUE:
        return IndefiniteTruthValue::fromString(tvStr);
        break;
    case COMPOSITE_TRUTH_VALUE:
        return CompositeTruthValue::fromString(tvStr);
        break;
    default:
        throw InvalidParamException(TRACE_INFO,
        		             "TruthValue - Invalid Truth Value type in factory(...): '%d'.", type);
        break;
    }
    return NULL;
}

	void TruthValue::DeleteAndSetDefaultTVIfPertinent(TruthValue** tv)
{
    if (*tv != &(TruthValue::DEFAULT_TV()) &&
            (*tv)->getType() == DEFAULT_TV().getType() &&
            (*tv)->getMean() == DEFAULT_TV().getMean() &&
            (*tv)->getCount() == DEFAULT_TV().getCount()) {
        delete *tv;
        *tv = (TruthValue*) & (DEFAULT_TV());
    }
}
