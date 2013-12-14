/*
 * opencog/atomspace/CompositeTruthValue.cc
 *
 * Copyright (C) 2008-2010 OpenCog Foundation
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
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

#include <stdlib.h>

#include <opencog/util/platform.h>

#include "CompositeTruthValue.h"
#include "AtomSpace.h"

//#define DPRINTF printf
#define DPRINTF(...)

using namespace opencog;

CompositeTruthValue::CompositeTruthValue(TruthValuePtr tv, VersionHandle vh)
{
    primaryTV = TruthValue::DEFAULT_TV();
    setVersionedTV(tv, vh);
}

CompositeTruthValue::CompositeTruthValue(CompositeTruthValue const& source)
{
    primaryTV = (source.primaryTV == TruthValue::DEFAULT_TV()) ?
        TruthValue::DEFAULT_TV() :
        source.primaryTV->clone();
    for (VersionedTruthValueMap::const_iterator itr = source.versionedTVs.begin();
            itr != source.versionedTVs.end(); ++itr)
    {
        VersionHandle vh = itr->first;
        TruthValuePtr tv = itr->second;
        versionedTVs[vh] = (tv == TruthValue::DEFAULT_TV()) ?
            TruthValue::DEFAULT_TV() :
            tv->clone();
    }
}

CompositeTruthValue::~CompositeTruthValue()
{
    primaryTV = NULL;
    versionedTVs.clear();
}

TruthValuePtr CompositeTruthValue::getPrimaryTV() const
{
    return primaryTV;
}

strength_t CompositeTruthValue::getMean() const
{
    return primaryTV->getMean();
}

count_t CompositeTruthValue::getCount() const
{
    return primaryTV->getCount();
}


confidence_t CompositeTruthValue::getConfidence() const
{
    return primaryTV->getConfidence();
}

static const char* typeToStr(TruthValueType t) throw (InvalidParamException)
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
             "Invalid Truth Value type: '%d'.", t);
    }
}

/*
 * The format of string representation of a Composite TV is as follows:
 * (primaryTv first, followed by the versionedTvs):
 * {FIRST_PLN_TRUTH_VALUE;[0.000001,0.500000=0.000625]}
 * {0x2;CONTEXTUAL;FIRST_PLN_TRUTH_VALUE;[0.500000,1.000000=0.001248]}
 * NOTE: string representation of tv types, VersionVandles and tv
 * attributes cannot have '{', '}' or ';', which are separators
 */
std::string CompositeTruthValue::toString() const
{
    DPRINTF("CompositeTruthValue::toString()\n");
    std::string result;
    char buffer[1<<16];
    DPRINTF("primaryTV = %p\n", primaryTV);
    DPRINTF("type = %d\n", primaryTV->getType());
    DPRINTF("typeStr = %s\n", typeToStr(primaryTV->getType()));
    DPRINTF("{%s;%s}", typeToStr(primaryTV->getType()), primaryTV->toString().c_str());
    sprintf(buffer, "{%s;%s}",
            typeToStr(primaryTV->getType()),
            primaryTV->toString().c_str());
    result += buffer;
    for (VersionedTruthValueMap::const_iterator itr = versionedTVs.begin();
         itr != versionedTVs.end(); ++itr)
    {
        VersionHandle key = itr->first;
        TruthValuePtr tv = itr->second;
        DPRINTF("{%p;%s;%s;%s}", key.substantive, VersionHandle::indicatorToStr(key.indicator), typeToStr(tv->getType()), tv->toString().c_str());
        sprintf(buffer, "{%lu;%s;%s;%s}",
                key.substantive.value(),
                VersionHandle::indicatorToStr(key.indicator),
                typeToStr(tv->getType()), tv->toString().c_str());
        result += buffer;
    }

    DPRINTF("\n%s\n", result.c_str());
    return result;
}

bool CompositeTruthValue::operator==(const TruthValue& rhs) const
{
    if (rhs.getType() != COMPOSITE_TRUTH_VALUE) return false;

    const CompositeTruthValue& crhs = 
           static_cast<const CompositeTruthValue&>(rhs);

    if (this == &crhs) return true;

    // Compare primary TV's
    if (primaryTV && !crhs.primaryTV) return false;
    if (!primaryTV && crhs.primaryTV) return false;
    if (primaryTV && *primaryTV != *crhs.primaryTV) return false;

    // Compare the versions
    if (getNumberOfVersionedTVs() != crhs.getNumberOfVersionedTVs()) return false;
    VersionedTruthValueMap::const_iterator itr;
    for (itr = versionedTVs.begin();
         itr != versionedTVs.end(); ++itr)
    {
        VersionHandle key = itr->first;
        VersionedTruthValueMap::const_iterator oitr;
        oitr = crhs.versionedTVs.find(key);
        if (oitr == crhs.versionedTVs.end()) return false;
        TruthValuePtr tv = itr->second;
        TruthValuePtr otv = oitr->second;
        if (*tv != *otv) return false;
    }

    // If we made it to here, the TV's must be identical.
    return true;  
}

TruthValueType CompositeTruthValue::getType() const
{
    return COMPOSITE_TRUTH_VALUE;
}


TruthValuePtr CompositeTruthValue::merge(TruthValuePtr other) const
{
    CompositeTruthValuePtr result = CompositeTVCast(clone());

    if (other->getType() == COMPOSITE_TRUTH_VALUE) {
        CompositeTruthValuePtr otherCTv = CompositeTVCast(other);

        if (otherCTv->getConfidence() > result->getConfidence()) {
            result->setVersionedTV(otherCTv->primaryTV, NULL_VERSION_HANDLE);
        }
        // merge the common versioned TVs
        for (VersionedTruthValueMap::const_iterator itr = result->versionedTVs.begin();
                itr != result->versionedTVs.end(); ++itr) {
            VersionHandle key = itr->first;
            TruthValuePtr tv = itr->second;
            VersionedTruthValueMap::const_iterator otherItr = otherCTv->versionedTVs.find(key);
            if (otherItr != otherCTv->versionedTVs.end()) {
                TruthValuePtr otherTv = otherItr->second;
                if (otherTv->getConfidence() > tv->getConfidence()) {
                    result->setVersionedTV(otherTv, key);
                }
            }
        }
        // adds the non-existing versioned TVs
        for (VersionedTruthValueMap::const_iterator otherItr = otherCTv->versionedTVs.begin();
                otherItr != otherCTv->versionedTVs.end(); ++otherItr) {
            VersionHandle key = otherItr->first;
            TruthValuePtr otherTv = otherItr->second;
            VersionedTruthValueMap::const_iterator itr = result->versionedTVs.find(key);
            if (itr == result->versionedTVs.end()) {
                result->setVersionedTV(otherTv, key);
            }
        }
    } else {
        if (other->getConfidence() > result->getConfidence()) {
            result->setVersionedTV(other, NULL_VERSION_HANDLE);
        }
    }
    return result;
}


void CompositeTruthValue::setVersionedTV(TruthValuePtr tv, VersionHandle vh)
{
    TruthValuePtr newTv = DEFAULT_TV();
    if (not tv->isNullTv() and tv != DEFAULT_TV()) {
        newTv = tv->clone();
    }
    VersionedTruthValueMap::const_iterator itr = versionedTVs.find(vh);
    if (itr == versionedTVs.end()) {
        if (!isNullVersionHandle(vh)) {
            // valid version handle
            versionedTVs[vh] = newTv;
        } else {
            // null version handle. Set the primary TV
            primaryTV = newTv;
        }
    }
    else {
        versionedTVs[vh] = newTv;
    }
}


TruthValuePtr CompositeTruthValue::getVersionedTV(VersionHandle vh) const
{
    if (!isNullVersionHandle(vh)) {
        VersionedTruthValueMap::const_iterator itr = versionedTVs.find(vh);
        if (itr == versionedTVs.end()) {
            return TruthValue::NULL_TV();
        } else {
            return (itr->second);
        }
    } else {
        return primaryTV;
    }
}

void CompositeTruthValue::removeVersionedTV(VersionHandle vh)
{
    VersionedTruthValueMap::const_iterator itr = versionedTVs.find(vh);
    if (itr != versionedTVs.end()) {
        TruthValuePtr versionedTv = itr->second;
        versionedTVs.erase(vh);
    }
}

void CompositeTruthValue::removeVersionedTVs(const Handle &substantive)
{
    VersionedTruthValueMap toBeRemovedEntries;

    VersionedTruthValueMap::const_iterator itr;
    for (itr = versionedTVs.begin();
         itr != versionedTVs.end(); ++itr)
    {
        VersionHandle key = itr->first;
        if (key.substantive == substantive)
        {
            toBeRemovedEntries[key] = NULL;
        }
    }
    for (itr = toBeRemovedEntries.begin();
         itr != toBeRemovedEntries.end(); ++itr)
    {
        VersionHandle key = itr->first;
        versionedTVs.erase(key);
    }
}

/**
 * Nearly identical to above, except tht all invalid handles are removed 
 */
void CompositeTruthValue::removeInvalidTVs(AtomSpace* atomspace)
{
    VersionedTruthValueMap toBeRemovedEntries;

    VersionedTruthValueMap::const_iterator itr;
    for (itr = versionedTVs.begin();
         itr != versionedTVs.end(); ++itr)
    {
        VersionHandle key = itr->first;
        if (!atomspace->isValidHandle(key.substantive))
        {
            toBeRemovedEntries[key] = NULL;
        }
    }
    for (itr = toBeRemovedEntries.begin();
         itr != toBeRemovedEntries.end(); ++itr)
    {
        VersionHandle key = itr->first;
        versionedTVs.erase(key);
    }
}


int CompositeTruthValue::getNumberOfVersionedTVs() const
{
    return versionedTVs.size();
}

VersionHandle CompositeTruthValue::getVersionHandle(int i) const
{
    int index = 0;
    for (VersionedTruthValueMap::const_iterator itr = versionedTVs.begin();
            itr != versionedTVs.end(); ++itr) {
        VersionHandle vh = itr->first;
        if (index++ == i) {
            return vh;
        }
    }
    return NULL_VERSION_HANDLE;
}

