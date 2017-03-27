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
    _importanceIndex(*this)
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

    _removeAtomConnection =
        asp->removeAtomSignal(
            boost::bind(&AttentionBank::remove_atom_from_index, this, _1));
}

AttentionBank::~AttentionBank()
{
    _removeAtomConnection.disconnect();
}

void AttentionBank::remove_atom_from_index(const AtomPtr& atom)
{
    std::unique_lock<std::mutex> lck(_idx_mtx);

    auto pr = _atom_index.find(Handle(atom));
    if (pr == _atom_index.end()) return;
    AttentionValuePtr av = pr->second;
    _atom_index.erase(pr->first);
    lck.unlock();

    int bin = ImportanceIndex::importanceBin(av->getSTI());

    _importanceIndex.removeAtom(atom.operator->(), bin);
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
    if (oldBin != newBin) updateImportanceIndex(h, oldBin, newBin);

    AVChanged(h, oldav, newav);
}

void AttentionBank::set_sti(const Handle& h, AttentionValue::sti_t stiValue)
{
    AttentionValuePtr old_av = AttentionValue::DEFAULT_AV();
    AttentionValuePtr new_av = createAV(
        stiValue, old_av->getLTI(), old_av->getVLTI());

    change_av(h, new_av);
}

void AttentionBank::set_lti(const Handle& h, AttentionValue::lti_t ltiValue)
{
    std::unique_lock<std::mutex> lck(_idx_mtx);

    AttentionValuePtr old_av = AttentionValue::DEFAULT_AV();
    auto pr = _atom_index.find(h);
    if (pr != _atom_index.end()) old_av = pr->second;

    AttentionValuePtr new_av = createAV(
        old_av->getSTI(), ltiValue, old_av->getVLTI());
    _atom_index[h] = new_av;
    lck.unlock();

    AVChanged(h, old_av, new_av);
}

void AttentionBank::change_vlti(const Handle& h, int unit)
{
    std::unique_lock<std::mutex> lck(_idx_mtx);

    AttentionValuePtr old_av = AttentionValue::DEFAULT_AV();
    auto pr = _atom_index.find(h);
    if (pr != _atom_index.end()) old_av = pr->second;

    AttentionValuePtr new_av = createAV(
        old_av->getSTI(),
        old_av->getLTI(),
        old_av->getVLTI() + unit);

    _atom_index[h] = new_av;
    lck.unlock();

    AVChanged(h, old_av, new_av);
}

AttentionValuePtr AttentionBank::get_av(const Handle& h)
{
    std::lock_guard<std::mutex> lck(_idx_mtx);
    auto pr = _atom_index.find(h);
    if (pr == _atom_index.end()) return AttentionValue::DEFAULT_AV();

    return pr->second;
}

void AttentionBank::AVChanged(const Handle& h,
                              const AttentionValuePtr& old_av,
                              const AttentionValuePtr& new_av)
{
    AttentionValue::sti_t oldSti = old_av->getSTI();
    AttentionValue::sti_t newSti = new_av->getSTI();
    
    // Add the old attention values to the AttentionBank funds and
    // subtract the new attention values from the AttentionBank funds
    updateSTIFunds(oldSti - newSti);
    updateLTIFunds(old_av->getLTI() - new_av->getLTI());

    // Update MinMax STI values
    AttentionValue::sti_t minSTISeen = 0;
    UnorderedHandleSet minbin = _importanceIndex.getMinBinContents();
    auto minit = std::min_element(minbin.begin(), minbin.end(),
            [&](const Handle& h1, const Handle& h2) {
                return get_sti(h1) < get_sti(h2);
            });
    if (minit != minbin.end()) minSTISeen = get_sti(*minit);

    AttentionValue::sti_t maxSTISeen = 0;
    UnorderedHandleSet maxbin = _importanceIndex.getMinBinContents();
    auto maxit = std::max_element(maxbin.begin(), maxbin.end(),
            [&](const Handle& h1, const Handle& h2) {
                return get_sti(h1) < get_sti(h2);
            });
    if (maxit != maxbin.end()) maxSTISeen = get_sti(*maxit);

    if (minSTISeen > maxSTISeen) {
        minSTISeen = maxSTISeen;
    }

    updateMinSTI(minSTISeen);
    updateMaxSTI(maxSTISeen);

    logger().fine("AVChanged: fundsSTI = %d, old_av: %d, new_av: %d",
                   fundsSTI.load(), oldSti, newSti);

    // Notify any interested parties that the AV changed.
    _AVChangedSignal(h, old_av, new_av);

    // Check if the atom crossed into or out of the AttentionalFocus
    // and notify any interested parties
    if (oldSti < getAttentionalFocusBoundary() and
        newSti >= getAttentionalFocusBoundary())
    {
        AFCHSigl& afch = AddAFSignal();
        afch(h, old_av, new_av);
    }
    else if (newSti < getAttentionalFocusBoundary() and
             oldSti >= getAttentionalFocusBoundary())
    {
        AFCHSigl& afch = RemoveAFSignal();
        afch(h, old_av, new_av);
    }
}

void AttentionBank::stimulate(const Handle& h, double stimulus)
{
    // XXX This is not protected or made atomic in any way ...
    // If two different threads stimulate the same atom at the same
    // time, then the calculations will be bad. Does it matter?
    AttentionValue::sti_t sti   = get_sti(h);
    AttentionValue::lti_t lti   = get_lti(h);
    AttentionValue::vlti_t vlti = get_vlti(h);

    AttentionValue::sti_t stiWage = calculateSTIWage() * stimulus;
    AttentionValue::lti_t ltiWage = calculateLTIWage() * stimulus;
    AttentionValuePtr new_av = createAV(sti + stiWage, lti + ltiWage, vlti);
    change_av(h, new_av);
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

Handle AttentionBank::getRandomAtom()
{
   return _importanceIndex.getRandomAtom();
}

