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
#include <opencog/atomspace/CompositeTruthValue.h>
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
    static TruthValuePtr instance(std::make_shared<SimpleTruthValue>(0, 0));
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
    static TruthValuePtr instance(std::make_shared<SimpleTruthValue>(MAX_TRUTH, 0.0));
    return instance;
}

TruthValuePtr TruthValue::merge(TruthValuePtr other) const
{
    if (other->getType() == COMPOSITE_TRUTH_VALUE) {
        return other->merge(clone());
    } else if (other->getConfidence() > getConfidence()) {
        return other->clone();
    }
    return clone();
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

TruthValuePtr TruthValue::factory(const char* fullTvStr)
{
    DPRINTF("TruthValue::factory(): fullTvStr = %s\n", fullTvStr);
    char* tvs = strdup(fullTvStr);
    char* typeStr = strtok(tvs, " \t\v\n");
    TruthValueType type = strToType(typeStr);
    size_t off = strlen(typeStr) + 1;
    free(tvs);
    return factory(type, fullTvStr + off);
}

TruthValuePtr TruthValue::factory(TruthValueType type, const char* tvStr)
    throw (InvalidParamException)
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

