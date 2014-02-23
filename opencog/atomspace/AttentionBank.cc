/*
 * opencog/atomspace/AttentionBank.cc
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
#include "AttentionBank.h"
#include "AtomTable.h"

using namespace opencog;

AttentionBank::AttentionBank(AtomTable* atab)
{
    startingFundsSTI = fundsSTI = config().get_int("STARTING_STI_FUNDS");
    startingFundsLTI = fundsLTI = config().get_int("STARTING_LTI_FUNDS");
    attentionalFocusBoundary = 1;

    AVChangedConnection = 
        atab->AVChangedSignal().connect(
            boost::bind(&AttentionBank::AVChanged, this, _1, _2, _3));
}

AttentionBank::~AttentionBank()
{
    AVChangedConnection.disconnect();
    fundsSTI = config().get_int("STARTING_STI_FUNDS");
}


void AttentionBank::AVChanged(Handle h, AttentionValuePtr old_av,
                                        AttentionValuePtr new_av)
{
    // Add the old attention values to the AtomSpace funds and
    // subtract the new attention values from the AtomSpace funds
    fundsSTI += (old_av->getSTI() - new_av->getSTI());
    fundsLTI += (old_av->getLTI() - new_av->getLTI());
}

long AttentionBank::getTotalSTI() const
{
    return startingFundsSTI - fundsSTI;
}

long AttentionBank::getTotalLTI() const
{
    return startingFundsLTI - fundsLTI;
}

long AttentionBank::getSTIFunds() const
{
    std::lock_guard<std::mutex> lock(lock_funds);
    return fundsSTI;
}

long AttentionBank::getLTIFunds() const
{
    std::lock_guard<std::mutex> lock(lock_funds);
    return fundsLTI;
}

long AttentionBank::updateSTIFunds(AttentionValue::sti_t diff)
{
    std::lock_guard<std::mutex> lock(lock_funds);
    fundsSTI += diff;
    return fundsSTI;
}

long AttentionBank::updateLTIFunds(AttentionValue::lti_t diff)
{
    std::lock_guard<std::mutex> lock(lock_funds);
    fundsLTI += diff;
    return fundsLTI;
}

void AttentionBank::updateMaxSTI(AttentionValue::sti_t m)
{
    std::lock_guard<std::mutex> lock(lock_maxSTI);
    maxSTI.update(m);
}

void AttentionBank::updateMinSTI(AttentionValue::sti_t m)
{
    std::lock_guard<std::mutex> lock(lock_minSTI);
    minSTI.update(m);
}

AttentionValue::sti_t AttentionBank::getMaxSTI(bool average) const
{
    std::lock_guard<std::mutex> lock(lock_maxSTI);
    if (average) {
        return (AttentionValue::sti_t) maxSTI.recent;
    } else {
        return maxSTI.val;
    }
}

AttentionValue::sti_t AttentionBank::getMinSTI(bool average) const
{
    std::lock_guard<std::mutex> lock(lock_minSTI);
    if (average) {
        return (AttentionValue::sti_t) minSTI.recent;
    } else {
        return minSTI.val;
    }
}

AttentionValue::sti_t AttentionBank::getAttentionalFocusBoundary() const
{
    return attentionalFocusBoundary;
}

AttentionValue::sti_t AttentionBank::setAttentionalFocusBoundary(AttentionValue::sti_t boundary)
{
    attentionalFocusBoundary = boundary;
    return boundary;
}

float AttentionBank::getNormalisedSTI(AttentionValuePtr av, bool average, bool clip) const
{
    // get normalizer (maxSTI - attention boundary)
    int normaliser;
    float val;
    AttentionValue::sti_t s = av->getSTI();
    if (s > getAttentionalFocusBoundary()) {
        normaliser = (int) getMaxSTI(average) - getAttentionalFocusBoundary();
        if (normaliser == 0) {
            return 0.0f;
        }
        val = (s - getAttentionalFocusBoundary()) / (float) normaliser;
    } else {
        normaliser = -((int) getMinSTI(average) + getAttentionalFocusBoundary());
        if (normaliser == 0) {
            return 0.0f;
        }
        val = (s + getAttentionalFocusBoundary()) / (float) normaliser;
    }
    if (clip) {
        return std::max(-1.0f, std::min(val,1.0f));
    } else {
        return val;
    }
}

float AttentionBank::getNormalisedZeroToOneSTI(AttentionValuePtr av, bool average, bool clip) const
{
    int normaliser;
    float val;
    AttentionValue::sti_t s = av->getSTI();
    normaliser = getMaxSTI(average) - getMinSTI(average);
    if (normaliser == 0) {
        return 0.0f;
    }
    val = (s - getMinSTI(average)) / (float) normaliser;
    if (clip) {
        return std::max(0.0f, std::min(val,1.0f));
    } else {
        return val;
    }
}

