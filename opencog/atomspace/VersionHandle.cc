/*
 * opencog/atomspace/VersionHandle.cc
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
#include <boost/functional/hash.hpp>

#include "VersionHandle.h"

#include <opencog/atomspace/TLB.h>
#include <opencog/util/exceptions.h>

using namespace opencog;

VersionHandle::VersionHandle()
{
    indicator = UNKNOWN;
    substantive = Handle::UNDEFINED;
}

VersionHandle::VersionHandle(IndicatorType ind, Handle subs)
{
    indicator = ind;
    substantive = subs;
}

VersionHandle::VersionHandle( const VersionHandle& other )
{
    this->indicator = other.indicator;
    this->substantive = other.substantive;
}

const char* VersionHandle::indicatorToStr(IndicatorType indicator) throw (InvalidParamException)
{
    switch (indicator) {
    case HYPOTHETICAL:
        return "HYPOTHETICAL";
    case CONTEXTUAL:
        return "CONTEXTUAL";
    case UNKNOWN:
        return "UNKNOWN";
    default:
        throw InvalidParamException(TRACE_INFO,
                                    "VersionHandle - Invalid indicator type: '%d'.", indicator);
    }
}

IndicatorType VersionHandle::strToIndicator(const char* indicatorStr) throw (InvalidParamException)
{
    for (int i = HYPOTHETICAL; i <= UNKNOWN; i++) {
        IndicatorType indicator = (IndicatorType) i;
        if (!strcmp(indicatorToStr(indicator), indicatorStr)) {
            return indicator;
        }
    }
    throw InvalidParamException(TRACE_INFO,
                                "VersionHandle - Invalid IndicatorType name: '%s'.", indicatorStr);
}

int hashVersionHandle::operator()(VersionHandle const vh) const
{
    int hashCode = (int) hash_value(vh);
    return(hashCode);
}

std::size_t opencog::hash_value(VersionHandle const& b)
{
    std::size_t seed = 0; 
    boost::hash_combine(seed,b.indicator);
    boost::hash_combine(seed,boost::hash<Handle>()(b.substantive));
    return seed;
}

bool eqVersionHandle::operator()(VersionHandle vh1, VersionHandle vh2) const
{
    return (vh1.indicator == vh2.indicator &&
            vh1.substantive == vh2.substantive);
}

bool VersionHandle::operator==(const VersionHandle &other) const
{
    struct eqVersionHandle comp;
    return comp(*this, other);
}

bool VersionHandle::operator!=(const VersionHandle &other) const
{
    struct eqVersionHandle comp;
    return !comp(*this, other);
}

bool VersionHandle::operator<(const VersionHandle &other) const
{
    // doesn't really matter
    return substantive < other.substantive;
}

bool VersionHandle::operator>(const VersionHandle &other) const
{
    // doesn't really matter
    return substantive > other.substantive;
}

VersionHandle& VersionHandle::operator=( const VersionHandle& other )
{
    this->indicator = other.indicator;
    this->substantive = other.substantive;
    return *this;
}
