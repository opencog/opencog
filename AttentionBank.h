/*
 * opencog/attentionbank/AttentionBank.h
 *
 * Copyright (C) 2011 OpenCog Foundation
 * Copyright (C) 2016, 2017 Linas Vepstas <linasvepstas@gmail.com>
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
#include <unordered_map>

#include <opencog/util/sigslot.h>
#include <opencog/truthvalue/AttentionValue.h>
#include <opencog/attentionbank/ImportanceIndex.h>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/* Attention Value changed */
typedef SigSlot<const Handle&,
                const AttentionValuePtr&,
                const AttentionValuePtr&> AVCHSigl;

/* Attentional Focus changed */
typedef SigSlot<const Handle&,
                const AttentionValuePtr&,
                const AttentionValuePtr&> AFCHSigl;

class AtomSpace;
class AttentionBank
{
    std::mutex _mtx; // For synchronizing STI & LTI funds update
    std::mutex AFMutex; // For AF fetching and update

    unsigned int maxAFSize;
    struct compare_sti_less {
        bool operator()(const std::pair<Handle, AttentionValuePtr>& h1,
                        const std::pair<Handle, AttentionValuePtr>& h2)
        {
            return  (h1.second)->getSTI() < (h2.second)->getSTI();
        }
    };
    std::multiset<std::pair<Handle, AttentionValuePtr>, compare_sti_less> attentionalFocus;

    void updateAttentionalFocus(const Handle&, const AttentionValuePtr&,
                                const AttentionValuePtr&);

    /** AV changes */
    void AVChanged(const Handle&, const AttentionValuePtr&, const AttentionValuePtr&);

    AtomPtrSignal* _remove_signal;
    int _remove_connection;

    /**
     * Signal emitted when an atom crosses in or out of the
     * AttentionalFocus.
     */
    AFCHSigl _AddAFSignal;
    AFCHSigl _RemoveAFSignal;

    AttentionValue::sti_t fundsSTI;
    AttentionValue::lti_t fundsLTI;

    AttentionValue::sti_t startingFundsSTI;
    AttentionValue::lti_t startingFundsLTI;

    AttentionValue::sti_t stiFundsBuffer;
    AttentionValue::lti_t ltiFundsBuffer;

    AttentionValue::sti_t targetSTI;
    AttentionValue::lti_t targetLTI;

    AttentionValue::sti_t STIAtomWage;
    AttentionValue::lti_t LTIAtomWage;

    /** The importance index */
    ImportanceIndex _importanceIndex;

    /** Signal emitted when the AV changes. */
    AVCHSigl _AVChangedSignal;

    void change_vlti(const Handle&, int);
    void remove_atom_from_bank(const AtomPtr& atom);

public:
    AttentionBank(AtomSpace*);
    ~AttentionBank();

#ifdef ECAN_EXPERIMENT
    std::map<Handle, AttentionValue::sti_t> stimulusRec;
#endif

    /**
     * Provide ability for others to find out about atoms that cross in or
     * out of the AttentionalFocus
     */
    AFCHSigl& AddAFSignal() { return _AddAFSignal; }
    AFCHSigl& RemoveAFSignal() { return _RemoveAFSignal; }

    /** Provide ability for others to find out about AV changes */
    AVCHSigl& getAVChangedSignal() { return _AVChangedSignal; }

    AttentionValue::sti_t get_af_max_sti(void) const
    {
        if (attentionalFocus.rbegin() != attentionalFocus.rend())
            return (attentionalFocus.rbegin()->second)->getSTI();
        else
            return 0;
    }

    AttentionValue::sti_t get_af_min_sti(void) const
    {
        if (attentionalFocus.rbegin() != attentionalFocus.rend())
            return (attentionalFocus.begin()->second)->getSTI();
        else
            return 0;
    }

    void set_af_size(int size) {
        maxAFSize = size;
    }

    int get_af_size(void) {
        return maxAFSize;
    }

    /**
     * Change the attention value of an atom.
     */
    void change_av(const Handle&, const AttentionValuePtr& new_av);
    void set_sti(const Handle&, AttentionValue::sti_t);
    void set_lti(const Handle&, AttentionValue::lti_t);
    void inc_vlti(const Handle& h) { change_vlti(h, +1); }
    void dec_vlti(const Handle& h) { change_vlti(h, -1); }


    /**
     * Stimulate an atom.
     *
     * @warning Should only be used by attention allocation system.
     * @param  h Handle to be stimulated
     * @param stimulus stimulus amount
     */
    void stimulate(const Handle&, double stimulus);

    /**
     * Get the total amount of STI in the AttentionBank, sum of
     * STI across all atoms.
     *
     * @return total STI in the AttentionBank
     */
    AttentionValue::sti_t getTotalSTI() const {
        return startingFundsSTI - (AttentionValue::sti_t)fundsSTI;
    }

    /**
     * Get the total amount of LTI in the AttentionBank, sum of
     * all LTI across atoms.
     *
     * @return total LTI in the AttentionBank
     */
    AttentionValue::lti_t getTotalLTI() const {
        return startingFundsLTI - (AttentionValue::lti_t)fundsLTI;
    }

    /**
     * Get the STI funds available in the AttentionBank pool.
     *
     * @return STI funds available
     */
    AttentionValue::sti_t getSTIFunds() const { return fundsSTI; }

    /**
     * Get the LTI funds available in the AttentionBank pool.
     *
     * @return LTI funds available
     */
    AttentionValue::lti_t getLTIFunds() const { return fundsLTI; }

    AttentionValue::sti_t getSTIFundsBuffer(){ return stiFundsBuffer;}

    AttentionValue::lti_t getLTIFundsBuffer(){ return ltiFundsBuffer;}

    AttentionValue::sti_t calculateSTIWage(void);

    AttentionValue::lti_t calculateLTIWage(void);

    /** Change the Very-Long-Term Importance of an attention value holder */
    //void setVLTI(AttentionValueHolderPtr, AttentionValue::vlti_t);

    /**
     * Retrieve the doubly normalised Short-Term Importance between -1..1
     * for a given AttentionValue. STI above and below threshold
     * normalised separately and linearly.
     *
     * @param h The attention value holder to get STI for
     * @param average Should the recent average max/min STI be used,
     *        or the exact min/max?
     * @param clip Should the returned value be clipped to -1..1?
     *        Outside this range can be return if average=true
     * @return normalised STI between -1..1
     */
    double getNormalisedSTI(AttentionValuePtr, bool average, bool clip) const;

    /**
     * @see getNormalisedSTI()
     */
    double getNormalisedSTI(AttentionValuePtr) const;

    /**
     * Retrieve the linearly normalised Short-Term Importance between 0..1
     * for a given AttentionValue.
     *
     * @param h The attention value holder to get STI for
     * @param average Should the recent average max/min STI be used,
     *        or the exact min/max?
     * @param clip Should the returned value be clipped to 0..1?
     *        Outside this range can be return if average=true
     * @return normalised STI between 0..1
     */
    double getNormalisedZeroToOneSTI(AttentionValuePtr, bool average, bool clip) const;

    bool atom_is_in_AF(const Handle&);

    /**
     * Gets the set of all handles in the Attentional Focus
     *
     * @return The set of all atoms in the Attentional Focus
     * @note: This method utilizes the ImportanceIndex
     */
    template <typename OutputIterator> OutputIterator
    get_handle_set_in_attentional_focus(OutputIterator result)
    {
         std::lock_guard<std::mutex> lock(AFMutex);
         for (const auto& p : attentionalFocus) {
             *result++ = p.first;
         }
         return result;
    }


    // =========================================================
    // Utility wrappers around the Importance Index.
    // XXX TODO -- Is this really needed? Users can operate thier
    // own importance index, if they need one, right?

    /// Return the index itself, giving direct access to it.
    ImportanceIndex& getImportance()
    {
        return _importanceIndex;
    }

    /// Return a random atom drawn from outside the AF.
    Handle getRandomAtomNotInAF(void);

    AttentionValue::sti_t getMinSTI(bool average=true) const
    {
        return _importanceIndex.getMinSTI(average);
    }

    AttentionValue::sti_t getMaxSTI(bool average=true) const
    {
        return _importanceIndex.getMaxSTI(average);
    }

    UnorderedHandleSet getHandlesByAV(AttentionValue::sti_t lowerBound,
                  AttentionValue::sti_t upperBound = AttentionValue::MAXSTI) const
    {
        return _importanceIndex.getHandleSet(lowerBound, upperBound);
    }

    /** Same as above, different API */
    template <typename OutputIterator> OutputIterator
    get_handles_by_AV(OutputIterator result,
                      AttentionValue::sti_t lowerBound,
                      AttentionValue::sti_t upperBound = AttentionValue::MAXSTI) const
    {
        UnorderedHandleSet hs = getHandlesByAV(lowerBound, upperBound);
        return std::copy(hs.begin(), hs.end(), result);
    }

};

/* Singleton instance (for now) */
AttentionBank& attentionbank(AtomSpace*);

/** @}*/
} //namespace opencog

#endif // _OPENCOG_ATTENTION_BANK_H
