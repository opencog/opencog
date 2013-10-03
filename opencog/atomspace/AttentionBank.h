/*
 * opencog/atomspace/AttentionBank.h
 *
 * Copyright (C) 2011 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Joel Pitt <joel@opencog.org>
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

#ifndef _OPENCOG_ATTENTION_BANK_H
#define _OPENCOG_ATTENTION_BANK_H

#include "AttentionValue.h"
#include <opencog/util/recent_val.h>

#include <boost/thread/mutex.hpp>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

class AttentionBank
{
    AtomSpaceImpl *atomspace;
    /**
     * Boundary at which an atom is considered within the attentional
     * focus of opencog. Atom's with STI less than this value are
     * not charged STI rent 
     */
    AttentionValue::sti_t attentionalFocusBoundary;

    opencog::recent_val<AttentionValue::sti_t> maxSTI;
    opencog::recent_val<AttentionValue::sti_t> minSTI;

    mutable boost::mutex lock_maxSTI;
    mutable boost::mutex lock_minSTI;

    /* These indicate the amount importance funds available in the
     * AtomSpace */
    long fundsSTI;
    long fundsLTI;

    long startingFundsSTI;
    long startingFundsLTI;

    mutable boost::mutex lock_funds;

    /**
     * Remove stimulus from atom, only should be used when Atom is deleted.
     */
    void removeStimulus(Handle h);

public:
    AttentionBank();
    ~AttentionBank();

    /**
     * Decays STI of all atoms in the AtomSpace (one cycle of importance decay).
     * @deprecated Importance updating should be done by ImportanceUpdating
     * Agent, but this is still used by Embodiment.
     */
    void decayShortTermImportance();

    /**
     * Get the total amount of STI in the AtomSpace, sum of
     * STI across all atoms.
     *
     * @return total STI in AtomSpace
     */
    long getTotalSTI() const;

    /**
     * Get the total amount of LTI in the AtomSpace, sum of
     * all LTI across atoms.
     *
     * @return total LTI in AtomSpace
     */
    long getTotalLTI() const;

    /**
     * Get the STI funds available in the AtomSpace pool.
     *
     * @return STI funds available
     */
    long getSTIFunds() const;

    /**
     * Get the LTI funds available in the AtomSpace pool.
     *
     * @return LTI funds available
     */
    long getLTIFunds() const;

    long updateSTIFunds(AttentionValue::sti_t diff);
    long updateLTIFunds(AttentionValue::lti_t diff);

    /**
     * Get attentional focus boundary, generally atoms below
     * this threshold won't be accessed unless search methods
     * are unsuccessful on those that are above this value.
     *
     * @return Short Term Importance threshold value
     */
    AttentionValue::sti_t getAttentionalFocusBoundary() const;

    /**
     * Change the attentional focus boundary. Some situations
     * may benefit from less focussed searches.
     *
     * @param s New threshold
     * @return Short Term Importance threshold value
     */
    AttentionValue::sti_t setAttentionalFocusBoundary(
        AttentionValue::sti_t s);

    /**
     * Get the maximum STI observed in the AtomSpace.
     *
     * @param average If true, return an exponentially decaying average of
     * maximum STI, otherwise return the actual maximum.
     * @return Maximum STI
     */
    AttentionValue::sti_t getMaxSTI(bool average=true) const;

    /**
     * Get the minimum STI observed in the AtomSpace.
     *
     * @param average If true, return an exponentially decaying average of
     * minimum STI, otherwise return the actual maximum.
     * @return Minimum STI
     */
    AttentionValue::sti_t getMinSTI(bool average=true) const;

    /**
     * Update the minimum STI observed in the connected AtomSpace. Min/max are not updated
     * on setSTI because average is calculate by lobe cycle, although this could
     * potentially also be handled by the cogServer.
     *
     * @warning Should only be used by attention allocation system.
     * @param m New minimum STI
     */
    void updateMinSTI(AttentionValue::sti_t m);

    /**
     * Update the maximum STI observed in the connected AtomSpace. Min/max are not updated
     * on setSTI because average is calculate by lobe cycle, although this could
     * potentially also be handled by the cogServer.
     *
     * @warning Should only be used by attention allocation system.
     * @param m New maximum STI
     */
    void updateMaxSTI(AttentionValue::sti_t m);

    /** Retrieve the AttentionValue of an attention value holder */
    const AttentionValue& getAV(AttentionValueHolderPtr avh) const;

    /** Change the AttentionValue of an attention value holder */
    void setAV(AttentionValueHolderPtr avh, const AttentionValue &av);

    /** Change the Short-Term Importance of an attention value holder */
    void setSTI(AttentionValueHolderPtr avh, AttentionValue::sti_t);

    /** Change the Long-term Importance of an attention value holder */
    void setLTI(AttentionValueHolderPtr avh, AttentionValue::lti_t);

    /** Change the Very-Long-Term Importance of an attention value holder */
    //void setVLTI(AttentionValueHolderPtr avh, AttentionValue::vlti_t);

    /** Incr the Very-Long-Term Importance of an attention value holder by 1*/
    void incVLTI(AttentionValueHolderPtr avh);

    /** Decr the Very-Long-Term Importance of an attention value holder by 1*/
    void decVLTI(AttentionValueHolderPtr avh);

    /** Retrieve the Short-Term Importance of an attention value holder */
    AttentionValue::sti_t getSTI(AttentionValueHolderPtr avh) const;

    /** Retrieve the Long-term Importance of a given Handle */
    AttentionValue::lti_t getLTI(AttentionValueHolderPtr avh) const;

    /** Retrieve the Very-Long-Term Importance of a given Handle */
    AttentionValue::vlti_t getVLTI(AttentionValueHolderPtr avh) const;

};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_ATTENTION_BANK_H

