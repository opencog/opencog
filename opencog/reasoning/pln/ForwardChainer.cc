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

#include "ForwardChainer.h"
#include "BackInferenceTreeNode.h"
#include "AtomSpaceWrapper.h"
#include "utils/NMPrinter.h"
#include "rules/Rules.h"

#include <opencog/util/Logger.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/util/iostreamContainer.h>

#include <boost/variant.hpp>
#include <time.h>

#include <algorithm>

using std::vector;
using std::set;
using std::cout;
using std::endl;
using std::set_union;

namespace haxx
{
    extern std::map<pHandle,std::vector<pHandle> > inferred_from;
    extern std::map<pHandle,opencog::pln::Rule*> inferred_with;
}

namespace opencog {
namespace pln {

//! @todo refactor into PLNUtils or maybe somewhere else.
void updateTrail(pHandle out, Rule* r, Btr<vector<BoundVertex> > args) {
    // Update the tacky trail mechanism (similar code to in the BIT)
    foreach(BoundVertex v, *args) {
        haxx::inferred_from[out].push_back(_v2h(v.value));
        haxx::inferred_with[out] = r;
    }
}

ForwardChainer::ForwardChainer(AtomSpaceWrapper* _asw) :
        composers(new ForwardComposerRuleProvider),
        generators(new ForwardGeneratorRuleProvider),
        asw(_asw)
{
    minConfidence = FWD_CHAIN_MIN_CONFIDENCE;
    probStack = FWD_CHAIN_PROB_STACK;
    probGlobal = FWD_CHAIN_PROB_GLOBAL;
}

ForwardChainer::~ForwardChainer()
{
    delete composers;
    delete generators;
}

pHandle ForwardChainer::fwdChainToTarget(int maxRuleApps, meta target)
{
    OC_ASSERT(target);
    // Should usually only be one (or none).
    pHandleSeq results = fwdChain(maxRuleApps, target);
    if (results.empty())
        return PHANDLE_UNDEFINED;
    else
        return results[0];
}

// Static/Shared random number generator
RandGen* ForwardChainer::rng = NULL;

RandGen* ForwardChainer::getRNG() {
    if (!rng)
        rng = new opencog::MT19937RandGen((unsigned long) time(NULL));
    return rng;
}

void ForwardChainer::printVertexVectorHandles(std::vector< Vertex > hs)
{
    printContainer(hs, ", ", "< ", " >");
}

//! @todo Find a good way to stop when it becomes only able to produce repeats.
//! @todo Possibly add an option for exhaustively applying Rules to all possible
//! inputs each step.
pHandleSeq ForwardChainer::fwdChain(int maxRuleApps, meta target)
{
    pHandleSeq results;
    
    while (maxRuleApps > 0) {
        //cout << "steps remaining: " << maxRuleApps << endl;
        bool appliedRule = false;
    
        // Get the next Rule (no restrictions)
        foreach(Rule *r, *composers) { // to avoid needing a nextRule method.
            cout << "Using " << r->getName() << endl;
        
            // Find the possible vector(s) of arguments for it
            std::set<std::vector<BBvtree> > filters(r->fullInputFilter());
            
            // For each such vector:
            foreach(std::vector<BBvtree> f, filters) {
                // find a vector of Atoms that matches it
                Btr<vector<BoundVertex> > args;

                Btr<set<Btr<vector<BoundVertex> > > > all_args;
                //args = findAllArgs(f);
                all_args = findAllArgs(f);
                bool foundArguments = !all_args->empty();

                // check for validity (~redundant)
                
                //cout << "FWDCHAIN arguments ";
                //printVertexVectorHandles(args); // takes Vertexes not BoundVertexes
                //cout << " are valid? " <<endl;
        
//                bool foundArguments = !args->empty();//true;// = r->validate(args);
        
                if (!foundArguments) {
                    //cout << "FWDCHAIN args not valid" << endl;
                }
                foreach(args, *all_args) {
                    if (args->empty()) {
                        cout << "*args empty. how?" << endl;
                        continue;
                    }
                    //cout << "FWDCHAIN args valid" << endl;
                
                    //args = *(all_args->begin());

                    // do the rule computation etc
                    
                    //! @todo Tacky check for Deduction. Equivalent to validate2, but that uses the other MP datatype,
                    //! which is why I didn't include it here.
                    if (r->getName() == std::string("DeductionRule")) {
//                        cout << "Deduction sanity check" << endl;
                        // A->B   B->C   check that those are not the same Link
                        bool valid = (args->size() == 2 && !((*args)[0].GetValue() == (*args)[1].GetValue()));
                        if (!valid) continue;
                    }

                    Vertex V=((r->compute(*args, PHANDLE_UNDEFINED, false)).GetValue());
                    pHandle out=boost::get<pHandle>(V);
                    const TruthValue& tv=GET_ASW->getTV(out);
                    //cout<<printTV(out)<<'\n';
                    NMPrinter().print(out,-5);

                    if (!tv.isNullTv() && tv.getCount() > minConfidence) {
						// If the TV is a repeat, delete it.
                    	// Ideally the chainer would now try another argument vector for this Rule
                    	// (until it uses them up).
                        vhpair v = GET_ASW->fakeToRealHandle(out);
                        //! @todo This check will no longer work.
                        if (! (v.second == NULL_VERSION_HANDLE)) {
                            GET_ASW->removeAtom(out);
                        } else {
                            maxRuleApps--;
                            appliedRule = true;

                            updateTrail(out, r, args);

                            if (target) { // Match it against the target
                                // Adapted from in Rule::validate
                                typedef weak_atom< meta > vertex_wrapper;
                                vertex_wrapper mp(target);

                                if (mp(out)) {
                                    results.push_back(out);

                                    return results;
                                }
                            } else
                                results.push_back(out);
                            //cout<<"Output\n";
                            NMPrinter np;
                            np.print(out);
                        }
                    } else {
                        // Remove atom if not satisfactory
                        //GET_ASW->removeAtom(_v2h(V));
                    }
                }
            }
        }
        // If it iterated through all Rules and couldn't find any suitable
        // input in the AtomSpace. Exit to prevent an infinite loop!
//! @todo Sometimes commented out as a hack, so it will keep going (potentially infinitely!) when a Rule fails the post-apply tests
        if (!appliedRule) return results;
    }
    
    return results;
}

//!@todo Remove the ones using non-primary TVs (only necessary while there's still the pHandle Hack).
Btr<std::set<BoundVertex> > ForwardChainer::getMatching(const meta target)
{
    Btr<std::set<BoundVertex> > matches(new std::set<BoundVertex>);
    
    cprintf(5, "getMatching: target: ");
    rawPrint(target->begin(), 5);

    foreach(Rule* g, *generators) {
    	//std::cout << g->getName();
    	Btr<std::set<BoundVertex> > gMatches = g->attemptDirectProduction(target);

    	if (gMatches.get()) {
            //foreach(BoundVertex tmp, *gMatches) matches->insert(tmp);

    	    // Since FC does not always provide adequate restrictions to
    	    // CCURule, it is necessary to do these checks on its output.
    	    // They are already done in Rule::compute on BoundVertexes, but
    	    // this way they will enable skipping problem Atoms rather than
    	    // causing assertions to fail.
    	    foreach(BoundVertex bv, *gMatches) {
    	        pHandle ph = _v2h(bv.value);
    	        //! @todo The first one may be an ASW / contexts issue.
                if  (ph != PHANDLE_UNDEFINED &&
                     !asw->isType(ph) &&
                     asw->getType(ph) != FW_VARIABLE_NODE) {
                    matches->insert(bv);
                } else {
                    cprintf(-1, "skipping invalid output from %s\n", g->getName().c_str());
                }
    	    }
    	}

    }
    
    return matches;
}

Btr<set<Btr<vector<BoundVertex> > > > ForwardChainer::findAllArgs(std::vector<BBvtree> filter)
{
    Btr<vector<BoundVertex> > args(new vector<BoundVertex>);
    Btr<set<Btr<vector<BoundVertex> > > > all_args(new set<Btr<vector<BoundVertex> > >);
    
    //Btr<bindingsT> bindings(new BindingsT());
    Btr<bindingsT> bindings(new std::map<pHandle, pHandle>);
    
    bool match = findAllArgs(filter, args, 0, all_args, bindings);

    return all_args;
}

bool ForwardChainer::findAllArgs(std::vector<BBvtree> filter, Btr<std::vector<BoundVertex> > args,
                                 uint current_arg, Btr<set<Btr<vector<BoundVertex> > > > all_args, Btr<bindingsT> bindings)
{
    bool exhaustive = false;
    // Sample about that many combinations with even probability.
    // Just checking if all_args < maxC' wouldn't be even.
    int maxCombinations = 100;
    int maxBranch = pow(maxCombinations, 1.0/filter.size());

    if (current_arg >= filter.size()) {
        // We've found a result, save it to all_args.
        // Need to copy args, because this method will change it immediately after.
        Btr<vector<BoundVertex> > args_copy(new vector<BoundVertex>);
        //(*args_copy) += (*args);
        foreach(BoundVertex tmp, *args)
            args_copy->push_back(tmp);
        all_args->insert(args_copy);
        //all_args->insert(args);

        return true;
    }
    
//    std::cout << "arg #" << current_arg << std::endl;
    
    BoundVertex bv;
    
    BBvtree f;
    f = filter[current_arg];
    // TODO also subst in any FWVar bindings so far
    //Btr<bindingsT> bindings(NULL); // should be a parameter to this method.
    
    //meta virtualized_target(bindings ? bind_vtree(*f,*bindings) : meta(new vtree(*f)));
    //meta virtualized_target(bindings ? bind_vtree(*f,*bindings) : meta(new vtree(*f)));
    meta virtualized_target(bind_vtree(*f,*bindings));
    //meta virtualized_target(meta(new vtree(*f)));
    ForceAllLinksVirtual(virtualized_target);
    
    Btr<std::set<BoundVertex> > choices;    
    choices = getMatching(virtualized_target);
    
    // pick one of the candidates at random to be the bv
    
//    std::cout << "choices size: " << choices->size() << std::endl;
    // random selection
    //! @todo sort based on strength and exponential random select
    //pHandle randArg;
    if (choices->size() == 0) {
//        std::cout << "backtracking" << std::endl;
        return false;
    }
    
    //std::cout << "choices: " << (*choices) << std::endl;

//    foreach (BoundVertex tmp, *choices)
//        std::cout << tmp.GetValue().which() << std::endl;


    // Alternative: is there some OpenCog library function to randomly select an item from a set?
    // @todo: can probably be simplified by using util/lazy_random_selector
    std::vector<BoundVertex> ordered_choices;
//    std::copy(choices->begin(), choices->end(), ordered_choices.begin());
    foreach (BoundVertex tmp, *choices) {
        //std::cout << _v2h(tmp.GetValue()) << std::endl;
        ordered_choices.push_back(tmp);
    }

    bool one_worked = false;

    //while ((ordered_choices.size() > 0) && (all_args.size() < maxCombinations)) {
    while ((ordered_choices.size() > 0) && (maxBranch-- > 0)) {
        int index = false ? 0
                               : getRNG()->randint(ordered_choices.size());
        //randArg = choices[index];
            
        bv = ordered_choices[index];
        ordered_choices.erase(ordered_choices.begin() + index);
        
        //choices.erase(choices.begin() + index);
        //args[i] = randArg;
        
        //boost::get<pHandle>(args[i]);
    
//        std::cout << "arg #" << current_arg << std::endl;
        NMPrinter np;
//        std::cout << "Using atom: " << std::endl;
        np.print(_v2h(bv.GetValue()));
    
        // Separate bindings before and after applying this, in case backtracking is necessary
        Btr<bindingsT> new_bindings(new std::map<pHandle, pHandle>);
        
        // Pass down all the bindings so far, plus any for this slot.
        foreach(hpair hp, *bindings)
            new_bindings->insert(hp);
        if (bv.bindings != NULL) {
            foreach(hpair hp, *bv.bindings) {
                new_bindings->insert(hp);
            }
        }
        
        //(*args)[current_arg] = bv;
        args->push_back(bv);
        
        // Whether the rest of the slots are successfully filled, when we use bv for this slot
        bool rest_filled = findAllArgs(filter, args, current_arg+1, all_args, new_bindings);
        if (rest_filled) {
            one_worked = true;
            if (!exhaustive)
                return true;
        }
        
        args-> pop_back();
//        else // tacky code to prevent more than one try
//            break;
    }
    
//    std::cout << "backtracking" << std::endl;
    
    //return false;
    return one_worked;
}

///////////////////////////////////////////////////////////////////////////////

HybridForwardChainer::HybridForwardChainer(AtomSpaceWrapper* _asw) :
        asw(_asw)
{
    minConfidence = FWD_CHAIN_MIN_CONFIDENCE;
    probStack = FWD_CHAIN_PROB_STACK;
    probGlobal = FWD_CHAIN_PROB_GLOBAL;

    ForwardChainer::composers = new DeductionRuleProvider;
    ForwardChainer::generators = new EvaluationRuleProvider;
}

HybridForwardChainer::~HybridForwardChainer()
{
}

//!@todo Remove the ones using non-primary TVs (only necessary while there's still the pHandle Hack).
//Btr<std::set<BoundVertex> > HybridForwardChainer::getMatching(const meta target)
//{
//    BITNodeRoot bit(target, new EvaluationRuleProvider);
//
//    int maxSteps = 1000;
//    const set<VtreeProvider*>& results =
//            bit.infer(maxSteps);
//
////    Btr<std::set<BoundVertex> > matches(new std::set<BoundVertex>);
////
////    cprintf(5, "getMatching: target: ");
////    rawPrint(target->begin(), 5);
////
////    foreach(Rule* g, *generators) {
////        //std::cout << g->getName();
////        Btr<std::set<BoundVertex> > gMatches = g->attemptDirectProduction(target);
////
////        if (gMatches.get()) {
////            //foreach(BoundVertex tmp, *gMatches) matches->insert(tmp);
////
////            // Since FC does not always provide adequate restrictions to
////            // CCURule, it is necessary to do these checks on its output.
////            // They are already done in Rule::compute on BoundVertexes, but
////            // this way they will enable skipping problem Atoms rather than
////            // causing assertions to fail.
////            foreach(BoundVertex bv, *gMatches) {
////                pHandle ph = _v2h(bv.value);
////                //! @todo The first one may be an ASW / contexts issue.
////                if  (ph != PHANDLE_UNDEFINED &&
////                     !asw->isType(ph) &&
////                     asw->getType(ph) != FW_VARIABLE_NODE) {
////                    matches->insert(bv);
////                } else {
////                    cprintf(-1, "skipping invalid output from %s\n", g->getName().c_str());
////                }
////            }
////        }
////
////    }
////
////    return matches;
//    Btr<std::set<BoundVertex> > ret;
//    //ret.reset(new std::set<BoundVertex>(results));
//    ret.reset(new std::set<BoundVertex>);
//
////    for (VTreeProvider* v, results) {
////        vtree& vt = v->getVtree();
////        BoundVertex bv(vt, )
////        ret->insert(v->)
////    }
//
//    foreach(VtreeProvider* vtp, results) {
//        ret->insert(BoundVertex(*vtp->getVtree().begin()));
//    }
//
//    //VtreeProviders_TO_BoundVertices(results, ret->begin());
//
//    return ret;
//}

Btr<set<Btr<vector<BoundVertex> > > > HybridForwardChainer::findAllArgs(std::vector<BBvtree> filter)
{
    // Make an ANDLink containing the arguments, and give it to the BC. Then convert each result into an arg-vector.
    // Used to not be able to make an ANDLink with 1 argument, because SimpleANDRule-1 crashed.
    //! @todo Streamline this code now.

    meta AND(
        new vtree(
             mva((pHandle)AND_LINK)
             )
    );

    for (unsigned int i = 0; i < filter.size(); i++)
    {
        //vtree arg(*filter[i]);
        AND->append_child(AND->begin(), filter[i]->begin());
    }

    // filter[0] is technically also a meta, i.e. Boost shared pointer to vtree.
    // Since BoundVTree is a subclass of vtree.
    BITNodeRoot bit(AND, new EvaluationRuleProvider);
    // Enables it to handle Deduction/MP combined with ForAll unification.
    bit.setLoosePoolPolicy(true);

    int maxSteps = 1000*filter.size();
    //const set<VtreeProvider*>& results =
    bit.infer(maxSteps);

    //! @todo Get the BIT to return results properly
    //! For now just using a workaround.

    bit.printResults();

    Btr<set<Btr<vector<BoundVertex> > > > ret(new set<Btr<vector<BoundVertex> > >);

    if (bit.getEvalResults().size()) {
        cout << bit.getEvalResults()[0].size() << endl;

        //results = bit.getEvalResults()[0];
    } else {
        return ret;
    }

    //foreach (VtreeProvider * vpt, results) {
    foreach (VtreeProvider * vpt, bit.getEvalResults()[0]) {
        Btr<vector<BoundVertex> > next(new vector<BoundVertex>);

        const vtree& tmp = vpt->getVtree();
        vtree::iterator top = tmp.begin();

        NMPrinter np;
        np.print(top,-5);

//            //np.print(top.begin(),-5);
//            //for (vtree::sibling_iterator i = tmp.begin(top); i!=tmp.end(top); i++) {
//            for (vtree::sibling_iterator i = top; i!=tmp.end(); i++) {
//                next->push_back(BoundVertex(*i));
//                np.print(i,-5);
//            }

        foreach (pHandle ph, asw->getOutgoing(_v2h(*top))) {
            next->push_back(BoundVertex(ph));
            np.print(ph,-5);
        }

        ret->insert(next);
    }

    return ret;
}



}} // namespace opencog::pln
