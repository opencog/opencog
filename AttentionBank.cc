/*
 * opencog/attentionbank/AttentionBank.cc
 *
 * Copyright (C) 2013,2017 Linas Vepstas <linasvepstas@gmail.com>
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

#include <functional>
#include <opencog/util/Config.h>

#include <opencog/atoms/base/Handle.h>
#include <opencog/atomspace/AtomSpace.h>
#include "AttentionBank.h"
#include "AVUtils.h"

using namespace opencog;

AttentionBank::AttentionBank(AtomSpace* asp)
{
    startingFundsSTI = fundsSTI = config().get_int("STARTING_STI_FUNDS", 100000);
    startingFundsLTI = fundsLTI = config().get_int("STARTING_LTI_FUNDS", 100000);
    stiFundsBuffer = config().get_int("STI_FUNDS_BUFFER", 10000);
    ltiFundsBuffer = config().get_int("LTI_FUNDS_BUFFER", 10000);
    targetLTI = config().get_int("TARGET_LTI_FUNDS", 10000);
    targetSTI = config().get_int("TARGET_STI_FUNDS", 10000);
    STIAtomWage = config().get_int("ECAN_STARTING_ATOM_STI_WAGE", 10);
    LTIAtomWage = config().get_int("ECAN_STARTING_ATOM_LTI_WAGE", 10);
    minAFSize = config().get_int("ECAN_MIN_AF_SIZE", 100);

    _remove_signal = &asp->atomRemovedSignal();
    _remove_connection = _remove_signal->connect(
            std::bind(&AttentionBank::remove_atom_from_bank, this,
                std::placeholders::_1));
}

AttentionBank::~AttentionBank()
{
    _remove_signal->disconnect(_remove_connection);
}

void AttentionBank::remove_atom_from_bank(const AtomPtr& atom)
{
    _importanceIndex.removeAtom(Handle(atom));
}

void AttentionBank::set_sti(const Handle& h, AttentionValue::sti_t stiValue)
{
    AttentionValuePtr oldav = get_av(h);
    AttentionValuePtr newav = AttentionValue::createAV(
        stiValue, oldav->getLTI(), oldav->getVLTI());

    _importanceIndex.updateImportance(h, oldav, newav);
    AVChanged(h, oldav, newav);
}

void AttentionBank::set_lti(const Handle& h, AttentionValue::lti_t ltiValue)
{
    AttentionValuePtr old_av = get_av(h);
    AttentionValuePtr new_av = AttentionValue::createAV(
        old_av->getSTI(), ltiValue, old_av->getVLTI());

    AVChanged(h, old_av, new_av);
}

void AttentionBank::change_vlti(const Handle& h, int unit)
{
    AttentionValuePtr old_av = get_av(h);
    AttentionValuePtr new_av = AttentionValue::createAV(
        old_av->getSTI(),
        old_av->getLTI(),
        old_av->getVLTI() + unit);

    AVChanged(h, old_av, new_av);
}

void AttentionBank::change_av(const Handle& h, const AttentionValuePtr& new_av)
{
    AttentionValuePtr old_av = get_av(h);
    _importanceIndex.updateImportance(h, old_av, new_av);
    AVChanged(h, old_av, new_av);
}

void AttentionBank::AVChanged(const Handle& h,
                              const AttentionValuePtr& old_av,
                              const AttentionValuePtr& new_av)
{
    // First, update the atom's actual AV.
    set_av(h, new_av);

    AttentionValue::sti_t oldSti = old_av->getSTI();
    AttentionValue::sti_t newSti = new_av->getSTI();

    // Add the old attention values to the AttentionBank funds and
    // subtract the new attention values from the AttentionBank funds
    updateSTIFunds(oldSti - newSti);
    updateLTIFunds(old_av->getLTI() - new_av->getLTI());

    _importanceIndex.update();

    logger().fine("AVChanged: fundsSTI = %d, old_av: %d, new_av: %d",
                   fundsSTI.load(), oldSti, newSti);

    // Notify any interested parties that the AV changed.
    _AVChangedSignal.emit(h, old_av, new_av);

    // update AF
    updateAttentionalFocus(h, old_av, new_av);
}

void AttentionBank::stimulate(const Handle& h, double stimulus)
{
    // XXX This is not protected or made atomic in any way ...
    // If two different threads stimulate the same atom at the same
    // time, then the calculations will be bad. Does it matter?
    AttentionValuePtr oldav(get_av(h));
    AttentionValue::sti_t sti   = oldav->getSTI();
    AttentionValue::lti_t lti   = oldav->getLTI();
    AttentionValue::vlti_t vlti = oldav->getVLTI();

    AttentionValue::sti_t stiWage = calculateSTIWage() * stimulus;
    AttentionValue::lti_t ltiWage = calculateLTIWage() * stimulus;
    AttentionValuePtr newav = AttentionValue::createAV(
           sti + stiWage, lti + ltiWage, vlti);
    _importanceIndex.updateImportance(h, oldav, newav);
    AVChanged(h, oldav, newav);
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
    if (s > get_af_max_sti()) {
        int normaliser = (int) getMaxSTI(average) - get_af_max_sti();
        if (normaliser == 0) return 0.0;
        val = (s - get_af_max_sti()) / (double) normaliser;
    } else {
        int normaliser = -((int) getMinSTI(average) + get_af_max_sti());
        if (normaliser == 0) return 0.0;
        val = (s + get_af_max_sti()) / (double) normaliser;
    }

    if (clip) return std::max(-1.0, std::min(val, 1.0));
    return val;
}

double AttentionBank::getNormalisedSTI(AttentionValuePtr av) const
{
    AttentionValue::sti_t s = av->getSTI();
    auto normaliser =
            s > get_af_max_sti() ? getMaxSTI() : getMinSTI();

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
// sort of more elegant way of managing the attentionbank is found.
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

bool AttentionBank::atom_is_in_AF(const Handle& h)
{
    auto it = std::find_if(attentionalFocus.begin(), attentionalFocus.end(),
            [h](std::pair<Handle, AttentionValuePtr> p)
            {
               if (p.first == h) return true;
               return false;
            });
    return  (it != attentionalFocus.end());
}

/**
 *  Updates list of top K important atoms based on STI value.
 */
void AttentionBank::updateAttentionalFocus(const Handle& h,
                    const AttentionValuePtr& old_av,
                    const AttentionValuePtr& new_av)
{
    std::lock_guard<std::mutex> lock(AFMutex);
    AttentionValue::sti_t sti = new_av->getSTI();
    auto least = attentionalFocus.begin(); // Atom to be removed from the AF
    bool insertable = false;
    auto it = std::find_if(attentionalFocus.begin(), attentionalFocus.end(),
            [h](std::pair<Handle, AttentionValuePtr> p)
            { if (p.first == h) return true; return false;});

    // Update the STI value if atoms was already in AF
    if (it != attentionalFocus.end())
    {
        attentionalFocus.erase(it);
        attentionalFocus.insert(std::make_pair(h, new_av));
        return;
    }

    // Simply insert the new Atom if AF is not full yet.
    else if (attentionalFocus.size() < minAFSize)
    {
        insertable = true;
    }

    // Remove the least sti valued atom in the AF and repace
    // it with the new atom holding higher STI value.
    else if (sti > (least->second)->getSTI())
    {
        Handle hrm = least->first;
        AttentionValuePtr hrm_new_av = get_av(hrm);
        // Value recorded when this atom entered into AF
        AttentionValuePtr hrm_old_av = least->second;

        attentionalFocus.erase(least);
        AFCHSigl& afch = RemoveAFSignal();
        afch.emit(hrm, hrm_old_av, hrm_new_av);
        insertable = true;
    }

    // Insert the new atom in to AF and emit the AddAFSignal.
    if (insertable)
    {
        attentionalFocus.insert(std::make_pair(h, new_av));
        AFCHSigl& afch = AddAFSignal();
        afch.emit(h, old_av, new_av);
    }
}
