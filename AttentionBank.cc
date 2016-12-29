/*
 * opencog/attentionbank/AttentionBank.cc
 *
 * Copyright (C) 2013 Linas Vepstas <linasvepstas@gmail.com>
 * All Rights Reserved
 *
 * Written by Joel Pitt
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

#include <boost/bind.hpp>
#include <opencog/util/Config.h>

#include <opencog/atoms/base/Handle.h>
#include <opencog/atomspace/AtomSpace.h>
#include "AttentionBank.h"

using namespace opencog;

AttentionBank::AttentionBank(AtomSpace* asp) :
    _index_insert_queue(this, &AttentionBank::put_atom_into_index, 4),
    _index_remove_queue(this, &AttentionBank::remove_atom_from_index, 4)
{
    startingFundsSTI = fundsSTI = config().get_int("STARTING_STI_FUNDS", 100000);
    startingFundsLTI = fundsLTI = config().get_int("STARTING_LTI_FUNDS", 100000);
    stiFundsBuffer = config().get_int("STI_FUNDS_BUFFER", 10000);
    ltiFundsBuffer = config().get_int("LTI_FUNDS_BUFFER", 10000);
    targetLTI = config().get_int("TARGET_LTI_FUNDS", 10000);
    targetSTI = config().get_int("TARGET_STI_FUNDS", 10000);
    STIAtomWage = config().get_int("ECAN_STARTING_ATOM_STI_WAGE", 10);
    LTIAtomWage = config().get_int("ECAN_STARTING_ATOM_LTI_WAGE", 10);

    _attentionalFocusBoundary = 1;

    // Subscribe to my own changes. This is insane and hacky; must move
    // the AV stuf out of the Atom itself.  XXX FIXME.
    _AVChangedConnection =
        _AVChangedSignal.connect(
            boost::bind(&AttentionBank::AVChanged, this, _1, _2, _3));

    bool async = config().get_bool("ATTENTION_BANK_ASYNC", false);
    if (async) {
     _addAtomConnection =
        asp->addAtomSignal(
            boost::bind(&AttentionBank::add_atom_to_indexInsertQueue, this, _1));
    _removeAtomConnection =
        asp->removeAtomSignal(
            boost::bind(&AttentionBank::add_atom_to_indexRemoveQueue, this, _1));
    }
    else {
     _addAtomConnection =
        asp->addAtomSignal(
            boost::bind(&AttentionBank::put_atom_into_index, this, _1));
    _removeAtomConnection =
        asp->removeAtomSignal(
            boost::bind(&AttentionBank::remove_atom_from_index, this, _1));

    }
}

void AttentionBank::change_av(const Handle& h, AttentionValuePtr newav)
{
    std::unique_lock<std::mutex> lck(_idx_mtx);

    AttentionValuePtr oldav = AttentionValue::DEFAULT_AV();
    auto pr = _atom_index.find(h);
    if (pr != _atom_index.end()) oldav = pr->second;

    _atom_index[h] = newav;
    lck.unlock();

    // XXX FIXME the code below could be made more efficient,
    // by avoiding the calls to atom->getAttentionValue() in
    // the ImportanceIndex code.

    // Get old and new bins.
    int oldBin = ImportanceIndex::importanceBin(oldav->getSTI());
    int newBin = ImportanceIndex::importanceBin(newav->getSTI());

    // If the atom importance has changed its bin,
    // update the importance index.
    if (oldBin != newBin) updateImportanceIndex(h, oldBin);

    // Notify any interested parties that the AV changed.
    _AVChangedSignal(h, oldav, newav);
}

AttentionValuePtr AttentionBank::get_av(const Handle& h)
{
    std::lock_guard<std::mutex> lck(_idx_mtx);
    auto pr = _atom_index.find(h);
    if (pr == _atom_index.end()) return AttentionValue::DEFAULT_AV();

    return pr->second;
}

AttentionBank::~AttentionBank()
{
    _AVChangedConnection.disconnect();
    _addAtomConnection.disconnect();
    _removeAtomConnection.disconnect();
}


void AttentionBank::AVChanged(const Handle& h,
                              const AttentionValuePtr& old_av,
                              const AttentionValuePtr& new_av)
{
    AttentionValue::sti_t newSti = new_av->getSTI();
    
    // Add the old attention values to the AttentionBank funds and
    // subtract the new attention values from the AttentionBank funds
    updateSTIFunds(old_av->getSTI() - newSti);
    updateLTIFunds(old_av->getLTI() - new_av->getLTI());

    // Update MinMax STI values
   UnorderedHandleSet minbin = _importanceIndex.getMinBinContents();
    AttentionValue::sti_t minSTISeen = (*std::min_element(minbin.begin(),minbin.end(),
            [](const Handle& h1, const Handle& h2) {
            return h1->getAttentionValue()->getSTI() < h2->getAttentionValue()->getSTI();
            }))->getAttentionValue()->getSTI();

    UnorderedHandleSet maxbin = _importanceIndex.getMaxBinContents();
    AttentionValue::sti_t maxSTISeen = (*std::max_element(maxbin.begin(),maxbin.end(),
            [](const Handle& h1, const Handle& h2) {
            return h1->getAttentionValue()->getSTI() > h2->getAttentionValue()->getSTI();
            }))->getAttentionValue()->getSTI();

    if (minSTISeen > maxSTISeen) {
        minSTISeen = maxSTISeen;
    }

    updateMaxSTI(maxSTISeen);
    updateMinSTI(minSTISeen);

    logger().fine("AVChanged: fundsSTI = %d, old_av: %d, new_av: %d",
                   fundsSTI.load(), old_av->getSTI(), new_av->getSTI());

    // Check if the atom crossed into or out of the AttentionalFocus
    // and notify any interested parties
    if (old_av->getSTI() < getAttentionalFocusBoundary() and
        new_av->getSTI() >= getAttentionalFocusBoundary())
    {
        AFCHSigl& afch = AddAFSignal();
        afch(h, old_av, new_av);
    }
    else if (new_av->getSTI() < getAttentionalFocusBoundary() and
             old_av->getSTI() >= getAttentionalFocusBoundary())
    {
        AFCHSigl& afch = RemoveAFSignal();
        afch(h, old_av, new_av);
    }
}

void AttentionBank::stimulate(const Handle& h, double stimulus)
{
    // XXX This is not protected and made atomic in any way ...
    // If two different threads stiumlate the same atom at the same
    // time, then the calculations could get wonky. Does it matter?
    AttentionValue::sti_t sti   = h->getAttentionValue()->getSTI();
    AttentionValue::lti_t lti   = h->getAttentionValue()->getLTI();
    AttentionValue::vlti_t vlti = h->getAttentionValue()->getVLTI();

    int stiWage = calculateSTIWage() * stimulus;
    int ltiWage = calculateLTIWage() * stimulus;

    AttentionValuePtr new_av = createAV(sti + stiWage, lti + ltiWage, vlti);
    h->setAttentionValue(new_av);
}

void AttentionBank::updateMaxSTI(AttentionValue::sti_t m)
{
    std::lock_guard<std::mutex> lock(_lock_maxSTI);
    _maxSTI.update(m);
}

void AttentionBank::updateMinSTI(AttentionValue::sti_t m)
{
    std::lock_guard<std::mutex> lock(_lock_minSTI);
    _minSTI.update(m);
}

AttentionValue::sti_t AttentionBank::getMaxSTI(bool average) const
{
    std::lock_guard<std::mutex> lock(_lock_maxSTI);
    if (average) {
        return (AttentionValue::sti_t) _maxSTI.recent;
    } else {
        return _maxSTI.val;
    }
}

AttentionValue::sti_t AttentionBank::getMinSTI(bool average) const
{
    std::lock_guard<std::mutex> lock(_lock_minSTI);
    if (average) {
        return (AttentionValue::sti_t) _minSTI.recent;
    } else {
        return _minSTI.val;
    }
}

AttentionValue::sti_t AttentionBank::calculateSTIWage()
{
    long funds = getSTIFunds();
    double diff  = funds - targetSTI;
    double ndiff = diff / stiFundsBuffer;
    ndiff = std::min(ndiff, 1.0);
    ndiff = std::max(ndiff, -1.0);

    return STIAtomWage + (STIAtomWage * ndiff);
}

AttentionValue::lti_t AttentionBank::calculateLTIWage()
{
    long funds = getLTIFunds();
    double diff  = funds - targetLTI;
    double ndiff = diff / ltiFundsBuffer;
    ndiff = std::min(ndiff, 1.0);
    ndiff = std::max(ndiff, -1.0);

    return LTIAtomWage + (LTIAtomWage * ndiff);
}

double AttentionBank::getNormalisedSTI(AttentionValuePtr av,
                                   bool average, bool clip) const
{
    double val;
    // get normalizer (maxSTI - attention boundary)
    AttentionValue::sti_t s = av->getSTI();
    if (s > getAttentionalFocusBoundary()) {
        int normaliser = (int) getMaxSTI(average) - getAttentionalFocusBoundary();
        if (normaliser == 0) return 0.0;
        val = (s - getAttentionalFocusBoundary()) / (double) normaliser;
    } else {
        int normaliser = -((int) getMinSTI(average) + getAttentionalFocusBoundary());
        if (normaliser == 0) return 0.0;
        val = (s + getAttentionalFocusBoundary()) / (double) normaliser;
    }

    if (clip) return std::max(-1.0, std::min(val, 1.0));
    return val;
}

double AttentionBank::getNormalisedSTI(AttentionValuePtr av) const
{
    AttentionValue::sti_t s = av->getSTI();
    auto normaliser =
            s > getAttentionalFocusBoundary() ? getMaxSTI() : getMinSTI();

    return (s / normaliser);
}

double AttentionBank::getNormalisedZeroToOneSTI(AttentionValuePtr av,
                                    bool average, bool clip) const
{
    AttentionValue::sti_t s = av->getSTI();
    int normaliser = getMaxSTI(average) - getMinSTI(average);
    if (normaliser == 0) return 0.0;

    double val = (s - getMinSTI(average)) / (double) normaliser;
    if (clip) return std::max(0.0, std::min(val, 1.0));
    return val;
}

/** Unique singleton instance (for now) */
// This implementation is pretty hokey, and is a stop-gap until some
// sort of moreelegant way of managing the attentionbank is found.
// One of the issues is that access via this function can be CPU-wasteful.
AttentionBank& opencog::attentionbank(AtomSpace* asp)
{
    static AttentionBank* _instance = nullptr;
    static AtomSpace* _as = nullptr;

    // Protect setting and getting against thread races.
    // This is probably not needed.
    static std::map<AtomSpace*, AttentionBank*> banksy;
    static std::mutex art;
    std::unique_lock<std::mutex> graffiti(art);

    if (_as != asp and _instance) {
        delete _instance;
        _instance = nullptr;
        _as = nullptr;
    }
    if (asp and nullptr == _instance) {
        _instance = new AttentionBank(asp);
        _as = asp;
    }
    return *_instance;
}
