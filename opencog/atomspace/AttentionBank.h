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

#include <mutex>

#include <boost/signals2.hpp>

#include <opencog/util/recent_val.h>
#include <opencog/atomspace/AttentionValue.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

typedef boost::signals2::signal<void (const Handle&,
                                      const AttentionValuePtr&,
                                      const AttentionValuePtr&)> AFCHSigl;

class AtomTable;

class AttentionBank
{
    /** The connection by which we are notified of AV changes */
    boost::signals2::connection AVChangedConnection;
    void AVChanged(Handle, AttentionValuePtr, AttentionValuePtr);

    /**
     * Boundary at which an atom is considered within the attentional
     * focus of opencog. Atom's with STI less than this value are
     * not charged STI rent 
     */
    AttentionValue::sti_t attentionalFocusBoundary;

    /** Signal emitted when an atom crosses in or out of the AttentionalFocus */
    AFCHSigl _AddAFSignal;
    AFCHSigl _RemoveAFSignal;

    opencog::recent_val<AttentionValue::sti_t> maxSTI;
    opencog::recent_val<AttentionValue::sti_t> minSTI;

    mutable std::mutex lock_maxSTI;
    mutable std::mutex lock_minSTI;

    /* These indicate the amount importance funds available in the
     * AtomSpace */
    long fundsSTI;
    long fundsLTI;

    long startingFundsSTI;
    long startingFundsLTI;

    mutable std::mutex lock_funds;

public:
    /** The table notifies us about AV changes */
    AttentionBank(AtomTable*);
    ~AttentionBank();

    /**
     * Provide ability for others to find out about atoms that cross in or
     * out of the AttentionalFocus
     */
    AFCHSigl& AddAFSignal() { return _AddAFSignal; }
    AFCHSigl& RemoveAFSignal() { return _RemoveAFSignal; }

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

    /** Change the Very-Long-Term Importance of an attention value holder */
    //void setVLTI(AttentionValueHolderPtr avh, AttentionValue::vlti_t);

    /** Retrieve the doubly normalised Short-Term Importance between -1..1
     * for a given AttentionValue. STI above and below threshold
     * normalised separately and linearly.
     *
     * @param h The attention value holder to get STI for
     * @param average Should the recent average max/min STI be used, or the
     * exact min/max?
     * @param clip Should the returned value be clipped to -1..1? Outside this
     * range can be return if average=true
     * @return normalised STI between -1..1
     */
    float getNormalisedSTI(AttentionValuePtr, bool average, bool clip) const;

    /** Retrieve the linearly normalised Short-Term Importance between 0..1
     * for a given AttentionValue.
     *
     * @param h The attention value holder to get STI for
     * @param average Should the recent average max/min STI be used, or the
     * exact min/max?
     * @param clip Should the returned value be clipped to 0..1? Outside this
     * range can be return if average=true
     * @return normalised STI between 0..1
     */
    float getNormalisedZeroToOneSTI(AttentionValuePtr, bool average, bool clip) const;
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_ATTENTION_BANK_H

