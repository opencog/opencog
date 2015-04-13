/*
 * opencog/dynamics/attention/SpreadDecider.h
 *
 * Copyright (C) 2008, 2014 OpenCog Foundation
 * Written by Joel Pitt <joel@fruitionnz.com>
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

#include "SpreadDecider.h"

namespace opencog
{

// Static/Shared random number generator
RandGen* SpreadDecider::rng = NULL;

bool SpreadDecider::spreadDecision(AttentionValue::sti_t s)
{
    if (getRNG()->randfloat() < function(s))
        return true;
    else
        return false;
}

RandGen* SpreadDecider::getRNG()
{
    if (!rng)
        rng = new opencog::MT19937RandGen((unsigned long) time(NULL));
    return rng;
}

double HyperbolicDecider::function(AttentionValue::sti_t s)
{
    AtomSpace& a = _cogserver.getAtomSpace();
    // Convert boundary from -1..1 to 0..1
    float af = a.getAttentionalFocusBoundary();
    float minSTI = a.getMinSTI(false);
    float maxSTI = a.getMaxSTI(false);
    float norm_b = focusBoundary > 0.0f ?
        af + (focusBoundary * (maxSTI - af)) :
        af + (focusBoundary * (af - minSTI ));
    // norm_b is now the actual STI value, normalise to 0..1
    norm_b = (norm_b - minSTI) / (float) ( maxSTI - minSTI );
    // Scale s to 0..1
    float norm_s = (s - minSTI) / (float) ( maxSTI - minSTI );
    return (tanh(shape*(norm_s-norm_b))+1.0f)/2.0f;
}

void HyperbolicDecider::setFocusBoundary(float b)
{
    // Store as -1..1 since exact 0..1 mapping of boundary
    // will change based on min/max STI
    focusBoundary = b;
}

double StepDecider::function(AttentionValue::sti_t s)
{
    return (s>focusBoundary ? 1.0f : 0.0f);
}

void StepDecider::setFocusBoundary(float b)
{
    AtomSpace& a = _cogserver.getAtomSpace();
    // Convert to an exact STI amount
    float af = a.getAttentionalFocusBoundary();
    focusBoundary = (b > 0.0f)?
        (int) (af + (b * (a.getMaxSTI(false) - af))) :
        (int) (af + (b * (af - a.getMinSTI(false))));

}

} // namespace
