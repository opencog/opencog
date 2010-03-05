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
#include "AtomSpaceWrapper.h"
#include "utils/NMPrinter.h"
#include "rules/Rules.h"

#include <opencog/util/Logger.h>
#include <opencog/util/mt19937ar.h>

#include <boost/variant.hpp>
#include <time.h>

using std::vector;
using std::cout;
using std::endl;

namespace opencog {
namespace pln {

ForwardChainer::ForwardChainer()
{
    float minConfidence = FWD_CHAIN_MIN_CONFIDENCE;
    float probStack = FWD_CHAIN_PROB_STACK; 
    float probGlobal = FWD_CHAIN_PROB_GLOBAL;
}

ForwardChainer::~ForwardChainer()
{
}

pHandle ForwardChainer::fwdChainToTarget(int& maxRuleApps, meta target)
{
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
    bool firstPrint = true;
    cout << "< ";
    foreach(Vertex v, hs) {
        if (firstPrint) {
            firstPrint = false;
        } else {
            cout << ", ";
        }
        //cout << boost::get<Handle>(v).value();
        cout << v;
    }
    cout << " >";
}

// Possibly should be elsewhere.
// Finds an input filter that has all the constraints, including between arguments.
// inputFilter doesn't include those constraints.
// Sometimes there is more than one input filter available (e.g. for SimSubstRule)
std::set<std::vector<BBvtree> > getFilters(Rule * r)
{
    //meta i2oType(const std::vector<Vertex>& h) const
    
    //! @todo extend this for other rules (probably separately for each one)
    // generic target for this rule
//    meta generic_target(new vtree(mva((pHandle)ASSOCIATIVE_LINK, 
//                                         vtree(CreateVar(GET_ASW)),
//                                         vtree(CreateVar(GET_ASW))
//                                         )));

    std::vector<meta> inputFilter(r->getInputFilter());
    
    // convert them to BoundVertex instances.
    // This should all probably be refactored.
    
    Btr<std::vector<BBvtree> > filter(new std::vector<BBvtree>);
    
    //std::copy(inputFilter.begin(), inputFilter.end(), filter->begin());
    
    foreach(meta item, inputFilter) {
        BBvtree Btr_bound_item(new BoundVTree(*item)); // not bound yet, it just means it can be bound
        filter->push_back(Btr_bound_item);
    }
    
    //return makeSingletonSet(filter);
    
    Rule::setOfMPs ret;
    ret.insert(*filter);
    return ret;
    

    meta generic_target(new vtree(mva((pHandle)ASSOCIATIVE_LINK, 
                                         mva((pHandle)ATOM),
                                         mva((pHandle)ATOM)
                                         )));

    //cout << "getFilters";
    rawPrint(generic_target->begin(), 2);
//    cout << rawPrint(*generic_target,generic_target->begin(),0);
    
    return r->o2iMeta(generic_target);
}

pHandleSeq ForwardChainer::fwdChain(int maxRuleApps, meta target)
{
    pHandleSeq results;
    
    while (maxRuleApps > 0) {
        //cout << "steps remaining: " << maxRuleApps << endl;        
        bool appliedRule = false;
    
        // Get the next Rule (no restrictions)
        foreach(Rule *r, rp) { // to avoid needing a nextRule method.
            //cout << "Using " << r->getName() << endl;
        
            // Find the possible vector(s) of arguments for it
            std::set<std::vector<BBvtree> > filters(r->fullInputFilter());
            
            // For each such vector:
            foreach(std::vector<BBvtree> f, filters) {
                // find a vector of Atoms that matches it
                Btr<vector<BoundVertex> > args;
                args = findAllArgs(f);
                // check for validity (~redundant)
                
                //cout << "FWDCHAIN arguments ";
                //printVertexVectorHandles(args); // takes Vertexes not BoundVertexes
                //cout << " are valid? " <<endl;
        
                bool foundArguments = !args->empty();//true;// = r->validate(args);
        
                if (!foundArguments) {
                    //cout << "FWDCHAIN args not valid" << endl;
                } else {
                    //cout << "FWDCHAIN args valid" << endl;
                
                    // do the rule computation etc
                    
                	//! @todo Tacky check for Deduction. Equivalent to validate2, but that uses the other MP datatype,
                	//! which is why I didn't includ it here.
                	if (r->getName() == std::string("DeductionRule")) {
                		cout << "Deduction sanity check" << endl;
                		// A->B   B->C   check that those are not the same Link
                		bool valid = (args->size() == 2 && !((*args)[0].GetValue() == (*args)[1].GetValue()));
                		if (!valid) continue;
                	}

                    Vertex V=((r->compute(*args, PHANDLE_UNDEFINED, false)).GetValue());
                    pHandle out=boost::get<pHandle>(V);
                    const TruthValue& tv=GET_ASW->getTV(out);
                    //cout<<printTV(out)<<'\n';

                    if (!tv.isNullTv() && tv.getCount() > minConfidence) {
						// If the TV is a repeat, delete it.
                    	// Ideally the chainer would now try another argument vector for this Rule
                    	// (until it uses them up).
						vhpair v = GET_ASW->fakeToRealHandle(out);
						if (! (v.second == NULL_VERSION_HANDLE)) {
							GET_ASW->removeAtom(out);
							continue;
						}
                        maxRuleApps--;
                        appliedRule = true;
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
                    } else {
                        // Remove atom if not satisfactory
                        //GET_ASW->removeAtom(_v2h(V));
                    }
                }
            }
        }
        // If it iterated through all Rules and couldn't find any suitable
        // input in the AtomSpace. Exit to prevent an infinite loop!
        if (!appliedRule) return results;
    }
    
    return results;
}

//!@todo Remove the ones using non-primary TVs (only necessary while there's still the pHandle Hack).
Btr<std::set<BoundVertex> > ForwardChainer::getMatching(const meta target)
{
    // Just look it up via LookupRule
    LookupRule lookup(GET_ASW);
    Btr<std::set<BoundVertex> > matches;
    
//    std::cout << "getMatching: target: ";
    rawPrint(target->begin(), 5);
                    
    matches = lookup.attemptDirectProduction(target);
    
    return matches;
}

Btr<vector<BoundVertex> > ForwardChainer::findAllArgs(std::vector<BBvtree> filter)
{
    Btr<vector<BoundVertex> > args(new vector<BoundVertex>);
    
    //Btr<bindingsT> bindings(new BindingsT());
    Btr<bindingsT> bindings(new std::map<pHandle, pHandle>);
    
    bool match = findAllArgs(filter, args, 0, bindings);

    return args;       
}

bool ForwardChainer::findAllArgs(std::vector<BBvtree> filter, Btr<std::vector<BoundVertex> > args,
                                 uint current_arg, Btr<bindingsT> bindings)
{
    if (current_arg >= filter.size())
        return true;
    
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
    std::vector<BoundVertex> ordered_choices;
//    std::copy(choices->begin(), choices->end(), ordered_choices.begin());
    foreach (BoundVertex tmp, *choices) {
        //std::cout << _v2h(tmp.GetValue()) << std::endl;
        ordered_choices.push_back(tmp);
    }

    while (ordered_choices.size() > 0) {        
        int index = (int) (getRNG()->randfloat() * ordered_choices.size() );
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
        foreach(hpair hp, *bv.bindings)
            new_bindings->insert(hp);
        
        
        //(*args)[current_arg] = bv;
        args->push_back(bv);
        
        // Whether the rest of the slots are successfully filled, when we use bv for this slot
        bool rest_filled = findAllArgs(filter, args, current_arg+1, new_bindings);
        if (rest_filled)
            return true; 
        
        args-> pop_back();
//        else // tacky code to prevent more than one try
//            break;
    }
    
//    std::cout << "backtracking" << std::endl;
    
    return false;
}

}} // namespace opencog::pln


