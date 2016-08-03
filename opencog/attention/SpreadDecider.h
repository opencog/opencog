/*
 * opencog/attention/SpreadDecider.h
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

#ifndef _OPENCOG_SPREAD_DECIDER_H
#define _OPENCOG_SPREAD_DECIDER_H

namespace opencog
{
/** \addtogroup grp_attention
 *  @{
 */

/** SpreadDeciders determiner whether a particular atom will diffuse STI
 */
class SpreadDecider
{
    //! This is the function that subclasses implement to determine
    //! the probility of whether an atom will spread importance based
    //! on its STI value.
    //!
    //! @param s The STI to base the decision on.
    //! @return Probability of given STI value resulting in spread.
    virtual double function(AttentionValue::sti_t) = 0;

protected:
    CogServer& _cogserver;
    static RandGen *rng;

public:
    SpreadDecider(CogServer& cs) : _cogserver(cs) {}
    virtual ~SpreadDecider() {}

    //! Set the focus boundary, which is used for centralising
    //! the the spread decision function. Note that this is in relation to the
    //! AtomSpace AttentionFocusBoundary as it's defined in terms of
    //! normalised STI.
    //!
    //! @param b the -1..1 normalised boundary position. 0 = AtomSpace
    //! AttentionalFocusBoundary.
    virtual void setFocusBoundary(double b = 0.0) {};

    //! Get the random number generator for the SpreadDecider
    //!
    //! @return pointer to the SpreadDecider RNG.
    RandGen* getRNG();

    //! This is interface to determine whether a certain STI value should 
    //! result in an atom spreading it's importance.
    //!
    //! @param s The STI to base the decision on.
    //! @return Whether to spread this importance.
    bool spreadDecision(AttentionValue::sti_t s);
};

/** Hyperbolic spread decider.
 * Uses the following function to return a probability of
 * spread when spreadDecision is called:
 * \f[
 *    P(\mathrm{spread}) = \frac{\mathrm{tanh}(shape\frac{STI_{0..1}}{b_{0..1}})+1}{2}
 * \f]
 * where \f$shape\f$ is the shape of the decision function, \f$STI_{0..1}\f$
 * and \f$b_{0..1}\f$ are the STI (for making a decision on) and the
 * focusBoundary (both normalised to the range \f$[0..1]\f$).
 */
class HyperbolicDecider : SpreadDecider
{
    //! Implements the hyperbolic function for returning a probability of
    //! spread.
    //!
    //! @param s The STI to base the decision on.
    //! @return Probability of given STI value resulting in spread.
    virtual double function(AttentionValue::sti_t s);

public:
    //! Used to determine the steepness of the hyperbolic decision function.
    double shape;

    //! The decision focus boundary, or where the decision function is centered
    //! around.
    double focusBoundary;

    HyperbolicDecider(CogServer& cs, double _s = 10.0):
        SpreadDecider(cs), shape(_s), focusBoundary(0.0) {}

    virtual ~HyperbolicDecider() {}
    virtual void setFocusBoundary(double b = 0.0);
};

/** A basic spread decider based on a step function.
 * I.e. any value above focusBoundary will return \f$P(\mathrm{spread}) = 1\f$.
 */
class StepDecider : SpreadDecider
{
    virtual double function(AttentionValue::sti_t s);

public:
    int focusBoundary;
    StepDecider(CogServer& cs) : SpreadDecider(cs) {}
    virtual ~StepDecider() {}
    virtual void setFocusBoundary(double b = 0.0);
};

/** @}*/
} // namespace

#endif // _OPENCOG_SPREAD_DECIDER_H
