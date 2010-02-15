/*
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

namespace opencog {
namespace pln {

const static int FWD_CHAIN_MAX_APPS = 100;
const static float FWD_CHAIN_MIN_CONFIDENCE = 0.4f;
const static float FWD_CHAIN_PROB_STACK = 0.7f;
const static float FWD_CHAIN_PROB_GLOBAL = 0.3f;
const static int FWD_CHAIN_MAX_FILL = 50;

//! Classes that manages forward chaining
class ForwardChainer {
private:

public:

    ForwardChainer();
    ~ForwardChainer();

    ForwardChainerRuleProvider rp;

    //! Rules that have yet to be attempted for forward chaining on seed handle.
    //! Consists of all rules at the beginning.
    std::deque<pHandle> seedStack;

    //! minimum confidence to accept a result or for using an atom
    float minConfidence;
    float probStack; 
    float probGlobal;
    //! In future perhaps support a higher chance for handles that have context
    //! specific information?
    //float probContext

    //! Chain to specific target. Only useful for testing.
    //! @param target Handle
    //! @param maximum number of rule applications before ending
    //! @param whether target was reached
    //Handle fwdChainToTarget(Handle target = Handle::UNDEFINED, int maxRuleApps = FWD_CHAIN_MAX_APPS);
    pHandle fwdChainToTarget(int& maxRuleApps, meta target);

    //! forward chain with the next Rule
    pHandleSeq fwdChain(int maxRuleApps = FWD_CHAIN_MAX_APPS, meta target = meta((vtree*)NULL));

    static RandGen* rng;
    RandGen* getRNG();

    //! Look up an atom to match the given target.
    //! Currently uses LookupRule; could evaluate ForAllLinks etc too (via the relevant rule).
    //! This would provide one way to actually choose the substitutions when using ForAlls.
    Btr<std::set<BoundVertex> > getMatching(const meta target);

    //! Find a series of atoms to match the given filter.
    Btr<std::vector<BoundVertex> > findAllArgs(std::vector<BBvtree> filter);
    
    // returns true if and only if all args from this point on bound successfully
    bool findAllArgs(std::vector<BBvtree> filter, Btr<std::vector<BoundVertex> > args,
                                 uint current_arg, Btr<bindingsT> bindings);

    void printVertexVectorHandles(std::vector< Vertex > hs);
};

}} // namespace opencog::pln
#endif // FORWARDCHAINER_H_
