/*
 * opencog/atomspace/CompositeTruthValue.h
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

#ifndef _OPENCOG__COMPOSITE_TRUTH_VALUE_H_
#define _OPENCOG__COMPOSITE_TRUTH_VALUE_H_

#include <functional>
#include <memory>
#include <unordered_map>

#include <opencog/util/functional.h>
#include <opencog/util/platform.h>

#include <opencog/atomspace/TruthValue.h>
#include <opencog/atomspace/VersionHandle.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class AtomSpace;

typedef std::unordered_map<VersionHandle, 
                             TruthValuePtr,
                             hashVersionHandle,
                             eqVersionHandle> VersionedTruthValueMap;

class CompositeTruthValue;
typedef std::shared_ptr<CompositeTruthValue> CompositeTruthValuePtr;

static inline CompositeTruthValuePtr CompositeTVCast(TruthValuePtr tv)
    { return std::dynamic_pointer_cast<CompositeTruthValue>(tv); }


class Atom;
class CompositeRenumber;

//! a TruthValue that consists of a number of VersionHandles paired with TruthValues
class CompositeTruthValue: public TruthValue
{
    friend class CompositeRenumber; // XXX ugly hack from file persistence

private:
    TruthValuePtr primaryTV;
    VersionedTruthValueMap versionedTVs;
public:
    /**
     * @param The initial primary or versioned TV of this composite TV.
     *        If it is NULL_TV(), a default tv will be created internally.
     * @param The VersionHandle the passed TV is associated to. For
     *        primary TV, its substantive component must be 
     *        Handle::UNDEFINED (you can use NULL_VERSION_HANDLE
     *        constant). In this case its indicator component does not
     *        matter.
     * @note  This object will take care of memory deallocation of the
     *        TruthValue object passed as argument to this method. So,
     *        the caller should not delete it outside.
     */
    CompositeTruthValue(TruthValuePtr, VersionHandle);
    CompositeTruthValue(CompositeTruthValue const&);

    ~CompositeTruthValue();

    /**
     * Merge this TV object with the given TV object argument.
     * It always returns a new TV object with the result of the merge,
     * even if it is equals to one of the merged TV objects.
     *
     * Currently tv1.merge(tv2) works as follows:
     * The resulting primaryTV is the primaryTV (of either tv1 or tv2) that has 
     * the highest confidence.
     * The resulting versionedTVs are the union of the versionedTVs of tv1 and
     * tv2, and for each versionedTV that is both in tv1 and tv2, then only
     * the one with the highest confidence is retained.
     */
    TruthValuePtr merge(TruthValuePtr) const;

    virtual bool operator==(const TruthValue& rhs) const;
    
    /**
     * @return The mean of the primary TruthValue
     */
    strength_t getMean() const;
    /**
     * @return the count of the primary TruthValue
     */
    count_t getCount() const;
    /**
     * @return the confidence of the primary TruthValue
     */
    confidence_t getConfidence() const;

    std::string toString() const;
    TruthValueType getType() const;

    /**
     * Sets the primary or a versioned TV in this CTV object.
     * @param TruthValue object to be set. If it is a NULL_TV(), a 
     *        default tv will be created internally.
     * @param The VersionHandle the passed TV is associated to. For
     *        the primary TV, its substantive component must be 
     *        Handle::UNDEFINED (you can use NULL_VERSION_HANDLE constant).
     *        In this case its indicator component does not matter.
     */
    void setVersionedTV(TruthValuePtr, VersionHandle);

    /**
     * Gets the versioned TruthValue object associated with the given
     * VersionHandle. If NULL_VERSION_HANDLE is given as argument,
     * returns the primaryTV.
     */
    TruthValuePtr getVersionedTV(VersionHandle) const;

    TruthValuePtr getPrimaryTV() const;

    /**
     * Removes the versioned TruthValue object associated with the
     * given VersionHandle.
     */
    void removeVersionedTV(VersionHandle);

    /**
     * Removes all versioned TruthValue objects associated with any
     * VersionHandle whose substantive component is equals to the
     * given Handle.
     */
    void removeVersionedTVs(const Handle &);

    /**
     * Removes all versioned TruthValue objects associated with any
     * VersionHandle whose substantive component is not a valid handle.
     * (i.e. whose corresponding atom has been deleted).
     *
     * @param atomspace The AtomSpace to check the handles against
     */
    void removeInvalidTVs(AtomSpace*);

    // clone method
    static CompositeTruthValuePtr createCTV(TruthValuePtr tv)
    {
        if (tv->getType() != COMPOSITE_TRUTH_VALUE)
            throw RuntimeException(TRACE_INFO, "Cannot clone non-composite TV");
        return std::make_shared<CompositeTruthValue>(
            static_cast<const CompositeTruthValue&>(*tv));
    }

    static TruthValuePtr createTV(TruthValuePtr tv)
    {
        return std::static_pointer_cast<TruthValue>(createCTV(tv));
    }

    TruthValuePtr clone() const
    {
        return std::make_shared<CompositeTruthValue>(*this);
    }

    TruthValue* rawclone() const
    {
        return new CompositeTruthValue(*this);
    }

    static CompositeTruthValuePtr createCTV(TruthValuePtr tv, VersionHandle vh)
    {
        return std::make_shared<CompositeTruthValue>(tv, vh);
    }

    static TruthValuePtr createTV(TruthValuePtr tv, VersionHandle vh)
    {
        return std::static_pointer_cast<TruthValue>(createCTV(tv, vh));
    }


    // iterator over VersionHandles
private:
    typedef select1st<VersionedTruthValueMap::value_type> get_key;
    typedef VersionedTruthValueMap::const_iterator vhm_const_iterator;
public:
    typedef boost::transform_iterator<get_key,
                                      vhm_const_iterator> vh_const_iterator;
    vh_const_iterator vh_begin() const {
        return boost::make_transform_iterator(versionedTVs.begin(), get_key());
    }
    vh_const_iterator vh_end() const {
        return boost::make_transform_iterator(versionedTVs.end(), get_key());
    }
    // helper for foreach
    std::pair<vh_const_iterator, vh_const_iterator> vh_range() const {
        return std::make_pair(vh_begin(), vh_end());
    }

    /**
     * Gets the number of versioned TVs of this CTV.
     */
    int getNumberOfVersionedTVs() const;

    /**
     * Gets the versioned TV given its index in the internal map.
     *
     * @note this method has complexity O(Index), if you want to do
     * that repetitively for all VersionedTV you should use
     * vh_begin(), vh_end() or vh_range().
     *
     * @param Index of the versioned TV. It must be a number between
     *        0 and (getNumberOfVersionedTVs()-1). Otherwise, it
     *        returns a NULL_VERSION_HANDLE.
     */
    VersionHandle getVersionHandle(int) const;
};

/** @}*/
} // namespace opencog

#endif // _OPENCOG__COMPOSITE_TRUTH_VALUE_H_
