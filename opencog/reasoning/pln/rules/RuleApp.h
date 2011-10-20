/*
 * Copyright (C) 2002-2007 Novamente LLC
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

#ifndef RULEAPP_H
#define RULEAPP_H

#include "Rule.h"
#include "RuleFunctions.h"
#include <opencog/atomspace/Atom.h>

/**
NOTE: RuleApps deliberately include NULL pointers in the arguments list, when dealing with Rules
with variable arity (flexible numbers of arguments). Seems to only be SimpleAndRule right now,
which doesn't even use flexible arities anyway? -- JaredW


BITNode class implements a compact inference tree representation, where each BITNode encapsulates
a Rule and
a vector of sets of other BITNode, where each set contains all the possible ways to get a result
for the argument slot in question.
Furthermore, BITNodes are parametrized, so that a single BITNode may stand for various inference
targets which differ only in the naming of the variables contained in them.

RuleApp class implements a comprehensive inference tree
representation, where you can know exactly which sequence of Rules and
intermediate results did produce a certain final result. (This is not
completely trivial, since it is not possible to store the explicit Rule
application trees for all the possible combinations of Rule applications
to tree depth N.)

In going backwards, then, we use BITNode to minimize the space
requirements for storing the trees.
In going forward, ie. applying some inference pathways present in
BITNode, we pass the results upward as RuleApp objects (whose
arguments lists are bound), rather than as direct Handles as before.

The RuleApp objects have 3 uses:

- Record the exact Rule application sequence. (previously this was done
using a cumbersome map structure)

- In a repository of (RuleApp, Resulting Handle, Resulting Handle's
fitness) tuples, one can easily mine for various statistically
interesting patterns of the form:
"what kinds of RuleApps are useful for producing what kinds of atoms?"

- A RuleApp can be used as a "macro Rule" which composes multiple
Rules into a single one. You can bind part of its arguments beforehand
with either vtrees (Handles) or with other RuleApps (some of which may
have complete and some incomplete argument lists). The RuleApp then
behaves like a Rule whose arity is the number of NULL arguments in all
the Rules it incorporates, and the ordering of the arguments starts from
the left-bottom-most Rule.

Eg. Dedu(Inv(A), Dedu($1, B)) takes one argument (bound to $1), whereas
Dedu(Inv($2), Dedu($1, B)) takes 2 args (1st one binds to $2, the 2nd
one to $1). Etc.


*/

namespace opencog { namespace pln {

struct VtreeProvider
{
    virtual const vtree& getVtree() const=0;
    virtual ~VtreeProvider() {};
    //virtual const* VtreeProvider clone() const=0;
};

struct VtreeProviderWrapper : public VtreeProvider
{
    vtree val;
    VtreeProviderWrapper(const vtree& _val) : val(_val) {}
    VtreeProviderWrapper(const Vertex& _val) : val(vtree(_val)) {}
    const vtree& getVtree() const { assert(this); return val; }
    virtual ~VtreeProviderWrapper() {};
    //const* VtreeProvider clone() const { return new VtreeProviderWrapper(val); }
};

template<typename VTPContainerT, typename OutIterT>
OutIterT VtreeProviders_TO_BoundVertices(const VTPContainerT& vtps, OutIterT out)
{
    foreach(VtreeProvider* vtp, vtps)
        (*out)++ = BoundVertex(*vtp->getVtree().begin());
    return out;
}

template<typename BVContainerT, typename OutIterT>
OutIterT BoundVertices_TO_VtreeProviders(const BVContainerT& bvs, OutIterT out)
{
    foreach(const BoundVertex& bv, bvs)
        (*out)++ = /*Btr<VtreeProvider*>*/(new VtreeProviderWrapper(bv.value));
    return out;
}

template<typename VContainerT, typename OutIterT>
OutIterT Vertices_TO_VtreeProviders(const VContainerT& vs, OutIterT out)
{
    foreach(const Vertex& v, vs)
        (*out)++ = /*Btr<VtreeProvider*>*/(new VtreeProviderWrapper(v));
    return out;
}

/**
	BITNode's arg results are stored as VtreeProviders.
	BITNode.compute() creates a RuleApp object out of its args.
	If result was not NULL, BITnode passes it upward as a RuleApp.
	If the parent BITNode wants to use the result for further computation,
	it just calls the result.getVtree(). 

	Specific successive Rule applications, ie. inference Pathways are, then,
	stored in RuleApps.
*/

/**
	With indefinite TVs, Rule.compute() can be called correctly with NULL args.
	Hence, RuleApp.compute() can always be called.
*/

class RuleApp : public VtreeProvider, public Rule
{
    mutable BoundVertex result;
    mutable vtree vt_result;
    mutable bool arg_changes_since_last_compute;
    mutable std::vector<VtreeProvider*> args;
    RulePtr root_rule;

    friend class BITNodeRoot;
public:
    virtual ~RuleApp();
    RuleApp(RulePtr _root_rule);
    
    /// Takes ownership of the "arg"
    /// false if arg was already bound. (And assert failure.)
    bool Bind(int arg_i, VtreeProvider* arg) const;
    
    /// false if arg was already bound.
    bool Bind(std::vector<VtreeProvider*>::iterator ai, VtreeProvider* arg) const;
    
    const vtree& getVtree() const;

    // From Rule
    std::set<MPs> o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

    bool validate2(MPs& _args) const { return true; }

    //NO_DIRECT_PRODUCTION; 
    Btr<std::set<BoundVertex > > attemptDirectProduction(meta outh,
                                                         bool fresh = true) const
    { 
        return Btr<std::set<BoundVertex> >(); 
    }

    /// For Rule interface.
    /// This method aborts with assert failure if the RuleApp is called
    /// with a wrong nr of args.
    BoundVertex compute(const VertexSeq& h,
                        pHandle CX = PHANDLE_UNDEFINED,
                        bool fresh = true) const;
    
    /// Use this when you know that all the args have already been Bound.
    /// The result is cached so the performance is unproblematic.
    BoundVertex compute(pHandle CX = PHANDLE_UNDEFINED,
                        bool fresh = true) const;
    //{
    //	vector<VtreeProvider*> dummy_vp;
    //	return compute(dummy_vp.end(), dummy_vp.end(), CX);
    //}
    
    template<typename IterT>
    BoundVertex compute(IterT begin, IterT end,
                        pHandle CX = PHANDLE_UNDEFINED,
                        bool fresh = true) const
    {
        IterT next_unused_arg;
        BoundVertex ret = compute(begin, end, next_unused_arg, CX, fresh);
        assert(next_unused_arg == end); // We don't allow args to remain unused ultimately.
        return ret;
    }

    /// Give the VtreeProvider* args in (begin, end). In the nextUnusedArg, we store the iterator
    /// that points to the first argument, of the given ones, which was not used.
    /// Note: An argument is not used if that same arg is already bound to a value.
    template<typename IterT, typename IterT2>
    BoundVertex compute(IterT begin, IterT end, IterT2& nextUnusedArg,
                        pHandle CX = PHANDLE_UNDEFINED,
                        bool fresh = true) const
    {
        std::vector<BoundVertex> bound_args;
        
        if (begin == end && !arg_changes_since_last_compute)
            {
                nextUnusedArg = end;
                goto out;
            }
        
        arg_changes_since_last_compute = false;
        
        /// Use the given args to bind recursively the open arg slots of the children,
        /// starting from the left-bottom-most one.
        /// If my arg_i is already filled in with a RuleApp, I pass the arg list to the RuleApp object.
        /// If my arg_i is already filled in with a vtree, I try my next arg slot.
        /// my arg_i is empty, I fill it with the given arg.
        
        nextUnusedArg = begin;
        
        for (std::vector<VtreeProvider*>::iterator ai = args.begin();
                ai != args.end(); ++ai) {
            BoundVertex bv;
            RuleApp* ra;

            if ((ra = dynamic_cast<RuleApp*>(*ai)) != NULL)
                //A bound a arg is a RuleApp
                bv = ra->compute(nextUnusedArg, end, nextUnusedArg, CX, fresh);
            else if (dynamic_cast<VtreeProviderWrapper*>(*ai) != NULL)
                //A bound arg has type VtreeProviderWrapper
                bv = *((*ai)->getVtree().begin());
            else { //A bound arg not found
                if (nextUnusedArg == end) //No more caller-supplied args available
#if FORMULA_CAN_COMPUTE_WITH_EMPTY_ARGS
                    assert(0); //Not implemented yet
#else
                {
                    if (!root_rule->hasFreeInputArity())
                    {
                        result = BoundVertex(PHANDLE_UNDEFINED);
                        goto out;
                    }
                    else // If no more arg information, and free input arity.
                    {
                        break;
                    }
                }
#endif

                bv = *(*(nextUnusedArg++))->getVtree().begin();
            }

#if !FORMULA_CAN_COMPUTE_WITH_EMPTY_ARGS
            if (bv.value == Vertex(PHANDLE_UNDEFINED))
            {
                nextUnusedArg = end;
                result = BoundVertex(PHANDLE_UNDEFINED);
                goto out;
            }
#endif

            //assert(v2h(bv.value)->isReal());
            assert(!GET_ASW->isType(boost::get<pHandle>(bv.value)));

            bound_args.push_back(bv);
        }

        result = root_rule->compute(bound_args, CX, fresh);

        /// This used to be below out, but when args are empty so is
        ///result.value and isReal is false
        assert(!GET_ASW->isType(boost::get<pHandle>(result.value)));
        out:

        return result;
    }
};

}} //namespace opencog { namespace pln {

#endif
