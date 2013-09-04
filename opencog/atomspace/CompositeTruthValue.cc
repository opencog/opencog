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

void CompositeTruthValue::init(const TruthValue& tv, VersionHandle vh)
{
    primaryTV = NULL;
    setVersionedTV(tv, vh);
    if (primaryTV == NULL) {
        primaryTV = (TruthValue*) & (TruthValue::DEFAULT_TV());
    }
}

void CompositeTruthValue::clear()
{
    if (primaryTV != &(TruthValue::DEFAULT_TV())) {
        delete primaryTV;
    }
    for (VersionedTruthValueMap::const_iterator itr = versionedTVs.begin();
            itr != versionedTVs.end(); ++itr) {
        TruthValue* tv = itr->second;
        if (tv != &(TruthValue::DEFAULT_TV())) {
            delete tv;
        }
    }
    versionedTVs.clear();
}

void CompositeTruthValue::copy(CompositeTruthValue const& source)
{
    primaryTV = (source.primaryTV == &(TruthValue::DEFAULT_TV())) ?
        (TruthValue*) & (TruthValue::DEFAULT_TV()) :
        source.primaryTV->clone();
    for (VersionedTruthValueMap::const_iterator itr = source.versionedTVs.begin();
            itr != source.versionedTVs.end(); ++itr) {
        VersionHandle vh = itr->first;
        TruthValue* tv = itr->second;
        versionedTVs[vh] = (tv == &(TruthValue::DEFAULT_TV())) ?
            (TruthValue*) & (TruthValue::DEFAULT_TV()) :
            tv->clone();
    }
}

CompositeTruthValue::CompositeTruthValue()
{
    primaryTV = NULL;
}

CompositeTruthValue::CompositeTruthValue(const TruthValue& tv, VersionHandle vh)
{
    init(tv, vh);
}

CompositeTruthValue::CompositeTruthValue(CompositeTruthValue const& source)
{
    copy(source);
}

CompositeTruthValue::~CompositeTruthValue()
{
    clear();
}

const TruthValue& CompositeTruthValue::getPrimaryTV() const
{
    return *primaryTV;
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

// Canonical methods
float CompositeTruthValue::toFloat() const
{
    // TODO: review this to consider versioned TVs as well?
    return primaryTV->toFloat();
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
    DPRINTF("typeStr = %s\n", TruthValue::typeToStr(primaryTV->getType()));
    DPRINTF("{%s;%s}", TruthValue::typeToStr(primaryTV->getType()), primaryTV->toString().c_str());
    sprintf(buffer, "{%s;%s}",
            TruthValue::typeToStr(primaryTV->getType()),
            primaryTV->toString().c_str());
    result += buffer;
    for (VersionedTruthValueMap::const_iterator itr = versionedTVs.begin();
         itr != versionedTVs.end(); ++itr) {
        VersionHandle key = itr->first;
        TruthValue* tv = itr->second;
        DPRINTF("{%p;%s;%s;%s}", key.substantive, VersionHandle::indicatorToStr(key.indicator), TruthValue::typeToStr(tv->getType()), tv->toString().c_str());
        sprintf(buffer, "{%lu;%s;%s;%s}",
                key.substantive.value(),
                VersionHandle::indicatorToStr(key.indicator),
                TruthValue::typeToStr(tv->getType()), tv->toString().c_str());
        result += buffer;
    }

    DPRINTF("\n%s\n", result.c_str());
    return result;
}

/**
 * The format of string representation of a Composite TV is as follows
 * (primaryTv first, followed by the versionedTvs):
 * {FIRST_PLN_TRUTH_VALUE;[0.000001,0.500000=0.000625]}
 * {0x2;CONTEXTUAL;FIRST_PLN_TRUTH_VALUE;[0.500000,1.000000=0.001248]}
 * NOTE: string representation of tv types, VersionHandles and tv 
 * attributes cannot have '{', '}' or ';', which are separators
 *
 * XXX This function should be moved to its own file-persistance 
 * directory. It really does not belong here, and things like the
 * UUID wackinesss is just potential for bugs.  In general, all of 
 * the file-persistence code should be removed from this directory.
 */
CompositeTruthValue* CompositeTruthValue::fromString(const char* tvStr) throw (InvalidParamException)
{
    char* buff;
    char* s = strdup(tvStr);
    // First each token is a tv representation
    char* tvToken = __strtok_r(s, "{}", &buff);
    if (tvToken == NULL) {
        throw InvalidParamException(TRACE_INFO,
            "Invalid string representation of a CompositeTruthValue object: "
            "missing primary TV!");
    }
    // Creates the new instance.
    CompositeTruthValue* result = new CompositeTruthValue();
    // Now, separate internal tokens
    char* internalBuff;
    char* primaryTvTypeStr = __strtok_r(tvToken, ";", &internalBuff);
    TruthValueType primaryTvType = TruthValue::strToType(primaryTvTypeStr);
    char* primaryTvStr = __strtok_r(NULL, ";", &internalBuff);
    DPRINTF("primary tvTypeStr = %s, tvStr = %s\n", primaryTvTypeStr, primaryTvStr);
    result->primaryTV = TruthValue::factory(primaryTvType, primaryTvStr);
    DeleteAndSetDefaultTVIfPertinent(&(result->primaryTV));

    // Get the versioned tvs
    while ((tvToken = __strtok_r(NULL, "{}", &buff)) != NULL) {
        char* substantiveStr = __strtok_r(tvToken, ";", &internalBuff);

        UUID uuid;
        sscanf(substantiveStr, "%lu", (UUID *) &uuid);
        Handle substantive(uuid);

        // TODO: IF THIS IS USED BY SAVING & LOADING, THIS HANDLE
        // MUST BE CONVERTED TO A NEW/COMMON HANDLE FORMAT.
        char* indicatorStr = __strtok_r(NULL, ";", &internalBuff);
        IndicatorType indicator = VersionHandle::strToIndicator(indicatorStr);
        DPRINTF("substantive = %p, indicator = %d\n", substantive.value(), indicator);
        char* versionedTvTypeStr = __strtok_r(NULL, ";", &internalBuff);
        TruthValueType versionedTvType = TruthValue::strToType(versionedTvTypeStr);
        char* versionedTvStr = __strtok_r(NULL, ";", &internalBuff);
        DPRINTF("tvTypeStr = %s, tvStr = %s\n", versionedTvTypeStr, versionedTvStr);
        VersionHandle vh(indicator, substantive);
        TruthValue* tv = TruthValue::factory(versionedTvType, versionedTvStr);
        DeleteAndSetDefaultTVIfPertinent(&tv);
        result->versionedTVs[vh] = tv;
    }
    free(s);
    return result;
}


CompositeTruthValue* CompositeTruthValue::clone() const
{
    return new CompositeTruthValue(*this);
}

CompositeTruthValue& CompositeTruthValue::operator=(const TruthValue & rhs) throw (RuntimeException)
{
    DPRINTF("CompositeTruthValue::operator=()\n");
    const CompositeTruthValue* tv = dynamic_cast<const CompositeTruthValue*>(&rhs);
    if (tv) {
        if (tv != this) { // check if this is the same object first.
            DPRINTF("operator=() calling clear()\n");
            clear();
            DPRINTF("operator=() calling copy()\n");
            copy((const CompositeTruthValue&) rhs);
        }
    } else {
#ifndef WIN32
        // The following line was causing a compilation error on MSVC...
        throw RuntimeException(TRACE_INFO, "Cannot assign a TV of type '%s' to one of type '%s'\n",
                               typeid(rhs).name(), typeid(*this).name());
#else
        throw RuntimeException(TRACE_INFO, "Invalid assignment of a CompositeTV object\n");
#endif
    }
    return *this;
}

CompositeTruthValue& CompositeTruthValue::operator=(const CompositeTruthValue & rhs) throw (RuntimeException)
{
    DPRINTF("CompositeTruthValue::operator=(const CompositeTruthValue&)\n");
    return operator=((const TruthValue&) rhs);
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
        TruthValue* tv = itr->second;
        TruthValue* otv = oitr->second;
        if (*tv != *otv) return false;
    }

    // If we made it to here, the TV's must be identical.
    return true;  
}

TruthValueType CompositeTruthValue::getType() const
{
    return COMPOSITE_TRUTH_VALUE;
}


TruthValue* CompositeTruthValue::merge(const TruthValue& other) const
{
    CompositeTruthValue* result = clone();
#if 1
    // TODO: Use the approach with dynamic cast below if we're going
    // to have subclasses of CompositeTruthValue. For now, this approach
    // using getType() is more efficient.
    if (other.getType() == COMPOSITE_TRUTH_VALUE) {
        const CompositeTruthValue* otherCTv = (CompositeTruthValue*) & other;
#else
    const CompositeTruthValue *otherCTv = dynamic_cast<const CompositeTruthValue *>(&other);
    if (otherCTv) {
#endif
        if (otherCTv->getConfidence() > result->getConfidence()) {
            result->setVersionedTV(*(otherCTv->primaryTV), NULL_VERSION_HANDLE);
        }
        // merge the common versioned TVs
        for (VersionedTruthValueMap::const_iterator itr = result->versionedTVs.begin();
                itr != result->versionedTVs.end(); ++itr) {
            VersionHandle key = itr->first;
            TruthValue* tv = itr->second;
            VersionedTruthValueMap::const_iterator otherItr = otherCTv->versionedTVs.find(key);
            if (otherItr != otherCTv->versionedTVs.end()) {
                TruthValue* otherTv = otherItr->second;
                if (otherTv->getConfidence() > tv->getConfidence()) {
                    result->setVersionedTV(*otherTv, key);
                }
            }
        }
        // adds the non-existing versioned TVs
        for (VersionedTruthValueMap::const_iterator otherItr = otherCTv->versionedTVs.begin();
                otherItr != otherCTv->versionedTVs.end(); ++otherItr) {
            VersionHandle key = otherItr->first;
            TruthValue* otherTv = otherItr->second;
            VersionedTruthValueMap::const_iterator itr = result->versionedTVs.find(key);
            if (itr == result->versionedTVs.end()) {
                result->setVersionedTV(*otherTv, key);
            }
        }
    } else {
        if (other.getConfidence() > result->getConfidence()) {
            result->setVersionedTV(other, NULL_VERSION_HANDLE);
        }
    }
    return result;
}


void CompositeTruthValue::setVersionedTV(const TruthValue& tv, VersionHandle vh) {

    TruthValue* newTv = (TruthValue*) & (TruthValue::DEFAULT_TV());
    if (!tv.isNullTv() && &tv != &(TruthValue::DEFAULT_TV())) {
        newTv = tv.clone();
    }
    VersionedTruthValueMap::const_iterator itr = versionedTVs.find(vh);
    if (itr == versionedTVs.end()) {
        if (!isNullVersionHandle(vh)) {
            // valid version handle
            versionedTVs[vh] = newTv;
        } else {
            // null version handle. Set the primary TV
            if (primaryTV != NULL && primaryTV != &(TruthValue::DEFAULT_TV())) {
                delete primaryTV;
            }
            primaryTV = newTv;
        }
    }
    else {
        TruthValue* versionedTv = itr->second;
        if (versionedTv != &(TruthValue::DEFAULT_TV())) {
            delete versionedTv;
        }
        versionedTVs[vh] = newTv;

    }
}


const TruthValue& CompositeTruthValue::getVersionedTV(VersionHandle vh) const {
    if (!isNullVersionHandle(vh)) {
        VersionedTruthValueMap::const_iterator itr = versionedTVs.find(vh);
        if (itr == versionedTVs.end()) {
            return TruthValue::NULL_TV();
        } else {
            return *(itr->second);
        }
    } else {
        return *primaryTV;
    }
}

void CompositeTruthValue::removeVersionedTV(VersionHandle vh)
{
    VersionedTruthValueMap::const_iterator itr = versionedTVs.find(vh);
    if (itr != versionedTVs.end()) {
        TruthValue* versionedTv = itr->second;
        versionedTVs.erase(vh);
        if (versionedTv != &(TruthValue::DEFAULT_TV())) {
            delete versionedTv;
        }
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

            // Free TruthValue object at once
            TruthValue* versionedTv = itr->second;
            if (versionedTv != &(TruthValue::DEFAULT_TV())) {
                delete versionedTv;
            }
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
            // Free TruthValue object at once
            TruthValue* versionedTv = itr->second;
            if (versionedTv != &(TruthValue::DEFAULT_TV())) {
                delete versionedTv;
            }
        }
    }
    for (itr = toBeRemovedEntries.begin();
         itr != toBeRemovedEntries.end(); ++itr)
    {
        VersionHandle key = itr->first;
        versionedTVs.erase(key);
    }
}


int CompositeTruthValue::getNumberOfVersionedTVs() const {
    return versionedTVs.size();
}

VersionHandle CompositeTruthValue::getVersionHandle(int i) const {
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

