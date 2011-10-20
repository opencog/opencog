/*
 * Copyright (C) 2008 by OpenCog Foundation
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

#ifndef FORWARDCHAINER_H_
#define FORWARDCHAINER_H_

#include "PLN.h"
#include "rules/RuleProvider.h"

#include <opencog/util/RandGen.h>
#include <opencog/util/Config.h>

namespace opencog {
namespace pln {

const static int FWD_CHAIN_MAX_APPS = 100;
const static float FWD_CHAIN_MIN_CONFIDENCE = 0.4f;
const static float FWD_CHAIN_PROB_STACK = 0.7f;
const static float FWD_CHAIN_PROB_GLOBAL = 0.3f;
const static int FWD_CHAIN_MAX_FILL = 50;

//! Forward chainer base class. Uses an obsolete approach that was not general and had an inappropriate combinatorial explosion.
//! See the more advanced HybridForwardChainer class. It still uses the fwdChain method here, which is the entry point for FC.
class ForwardChainer {
private:

public:

    ForwardChainer(AtomSpaceWrapper* _asw = GET_ASW);
    ~ForwardChainer();

    RuleProvider* composers;
    RuleProvider* generators;

    AtomSpaceWrapper* asw;

    //! Rules that have yet to be attempted for forward chaining on seed handle.
    //! Consists of all rules at the beginning.
    std::deque<pHandle> seedStack;

    //! minimum confidence to accept a result or for using an atom
    float minConfidence;
    float probStack; //@todo:??
    float probGlobal; //@todo:??
    //! In future perhaps support a higher chance for handles that have context
    //! specific information?
    //float probContext

    //! Store the results from the last level of the inference (and on the next step, you need to use at least
    //! one of them in each inference. This (more-or-less) avoids rerunning inferences.
    Btr<pHandleSet> lastLevelResults;
    Btr<pHandleSet> thisLevelResults;
    int level;

    //! @todo: probably to be removed
    //! Chain to specific target. Only useful for testing.
    //! @param target Handle
    //! @param maximum number of rule applications before ending
    //! @param whether target was reached
    //Handle fwdChainToTarget(Handle target = Handle::UNDEFINED, int maxRuleApps = FWD_CHAIN_MAX_APPS);

    //! Chain to specific target. Only useful for testing.
    //!
    //! @param maxRuleApps number of rule applications before ending
    //! @param target target to reach (assumed not to be null)
    //! @return the pHandle of the target if reached, else PHANDLE_UNDEFINED
    pHandle fwdChainToTarget(int maxRuleApps, meta target);

    //! Apply forward chaining till the target is reached, or if null,
    //! till the number of rule applications is maxRuleApps.
    //! 
    //! @param maxRuleApps number of rule applications before ending
    //! @param target target to reach (if NULL then no target)
    //! @return pHandles corresponding to all inferred atoms if meta is NULL
    //!         if meta is not NULL, then only the target if reached is returned
    pHandleSeq fwdChain(int maxRuleApps = FWD_CHAIN_MAX_APPS, meta target = meta((vtree*)NULL));

    static RandGen* rng;
    RandGen* getRNG();

    //! Look up an atom to match the given target.
    //! Currently uses LookupRule; could evaluate ForAllLinks etc too (via the relevant rule).
    //! This would provide one way to actually choose the substitutions when using ForAlls.
    virtual Btr<std::set<BoundVertex> > getMatching(const meta target);

    //! Find a series of atoms to match the given filter.
    virtual Btr<std::set<Btr<std::vector<BoundVertex> > > > findAllArgs(std::vector<BBvtree> filter);
    
    // returns true if and only if all args from this point on bound successfully
    bool findAllArgs(std::vector<BBvtree> filter, Btr<std::vector<BoundVertex> > args,
                                 uint current_arg,
                                 Btr<std::set<Btr<std::vector<BoundVertex> > > > all_args,
                                 Btr<bindingsT> bindings);

    void printVertexVectorHandles(std::vector< Vertex > hs);
};

// ForwardChainer subclass that uses similar inference control to a classical logic FC,
// i.e. it uses a deduction-ish Rule each step, and evaluates any And/Or/etc expressions
// in the ImplicationLink/etc. It uses the backward chainer to evaluate the expressions.
// Example step: applying ModusPonens on an ImplicationLink (produces B given A and {A implies B}).
// It would call the BC to find all ImplicationLinks, and also the antecedent (A here).
// The BC will return all ImplicationLinks where the antecedent is known already. The resulting
// conclusions may be useful in future FC steps.
class HybridForwardChainer : public ForwardChainer {
private:

    int bcSteps; //! The number of BC steps to do to find (all) required arguments for a Rule during an FC step

public:
    AtomSpaceWrapper* asw;

    HybridForwardChainer(AtomSpaceWrapper* _asw = GET_ASW);
    ~HybridForwardChainer();

    // Only enable one of the following.

    //! Find a series of atoms to match the given filter.
    //! Uses a single BIT (backward chaining search tree) to fill the whole filter (which should allow extra
    //! results in the ForAll case, and be more efficient). The BIT is configured to only use evaluation-ish
    //! Rules and not deduction-ish Rules.
    virtual Btr<std::set<Btr<std::vector<BoundVertex> > > > findAllArgs(std::vector<BBvtree> filter);

    //! Look up an atom to match the given target.
    //! Uses a BIT.
    //virtual Btr<std::set<BoundVertex> > getMatching(const meta target);
};

}} // namespace opencog::pln
#endif // FORWARDCHAINER_H_
