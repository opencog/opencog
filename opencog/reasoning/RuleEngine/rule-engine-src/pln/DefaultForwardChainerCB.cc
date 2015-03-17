/*
 * DefaultForwardChainerCB.cc
 *
 * Copyright (C) 2015 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>
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

#include "DefaultForwardChainerCB.h"
#include "PLNCommons.h"

#include <opencog/query/PatternMatch.h>
#include <opencog/guile/SchemeSmob.h>
#include <opencog/guile/SchemeEval.h>

DefaultForwardChainerCB::DefaultForwardChainerCB(AtomSpace* as) :
        ForwardChainerCallBack(as)
{
    as_ = as;
    fcim_ = new ForwardChainInputMatchCB(as);
    fcpm_ = new ForwardChainPatternMatchCB(as);
}

DefaultForwardChainerCB::~DefaultForwardChainerCB()
{
    delete fcim_;
    delete fcpm_;
}

/**
 * choose rule based on premises of rule matching the target
 * uses temporary atomspace to limit the search space and avoid
 *
 * @return a vector of chosen rules
 **/
//xxx this method uses SchemeEval and SchemeSmob for transferring handles
//from main atomspace to temporary atomspace.I tried to use AddAtom but
//it was not working.Handles were not being passed.So I ended up using SchemeEval
//with a hack (delete previously created SchemEval instance in order to use SchemeEval
//with another atomspace)
vector<Rule*> DefaultForwardChainerCB::choose_rule(FCMemory& fcmem)
{
    Handle target = fcmem.get_cur_target();
    if (target == Handle::UNDEFINED or NodeCast(target))
        throw InvalidParamException(TRACE_INFO,
                                    "Needs a target atom of type LINK");
    HandleSeq chosen_bindlinks;

    if (LinkCast(target)) {
        AtomSpace rule_atomspace;
        SchemeEval* sceval = new SchemeEval(&rule_atomspace);
        //Handle target_cpy=rule_atomspace.addAtom(target); xxx this doesn't work
        Handle target_cpy = sceval->eval_h(SchemeSmob::to_string(target)); //xxx this works

        //copy rules to the temporary atomspace
        vector<Rule*> rules = fcmem.get_rules();
        for (Rule* r : rules) {
            //rule_atomspace.addAtom(r->get_handle()); xxx this doesn't work
            sceval->eval_h(SchemeSmob::to_string(r->get_handle())); //xxx this works
        }

        //create bindlink with target as an implicant
        PLNCommons pc(&rule_atomspace);
        Handle copy = pc.replace_nodes_with_varnode(target_cpy, NODE);
        Handle bind_link = pc.create_bindLink(copy, false);

        //pattern match
        DefaultImplicator imp(&rule_atomspace);
        try {
            PatternMatch pm;
            pm.do_bindlink(bind_link, imp);
        } catch (InvalidParamException& e) {
            cout << "VALIDATION FAILED:" << endl << e.what() << endl;
        }

        //get matched bindLinks
        HandleSeq matches = imp.result_list;

        if (matches.empty()) {
            logger().debug(
                    "No matching BindLink was found.Returning empty vector");
            return vector<Rule*> { };
        }

        HandleSeq bindlinks;
        for (Handle hm : matches) {
            HandleSeq hs = get_rootlinks(hm, &rule_atomspace, BIND_LINK);
            for (Handle hi : hs) {
                if (find(bindlinks.begin(), bindlinks.end(), hi) == bindlinks.end()) {
                    bindlinks.push_back(hi);
                }
            }
        }
        delete sceval; //delete to use SchemeEval with another atomspace.this might be a hack.

        //push back handles to main atomspace
        for (Handle h : bindlinks) {
            //auto bindlink = as_->addAtom(h);
            SchemeEval *sc = new SchemeEval(as_);
            auto bindlink = sc->eval_h(SchemeSmob::to_string(h));
            chosen_bindlinks.push_back(bindlink);
        }
    }

    //trying to find specialized rules that contain the target node
    if (NodeCast(target)) {
        chosen_bindlinks = get_rootlinks(target, as_, BIND_LINK);
    }

    //find the rules containing the bindLink in copied_back
    vector<Rule*> matched_rules;
    vector<Rule*> rules = fcmem.get_rules();
    for (Rule* r : rules) {
        auto it = find(chosen_bindlinks.begin(), chosen_bindlinks.end(),
                       r->get_handle()); //xxx not matching
        if (it != chosen_bindlinks.end()) {
            cout << "RULE FOUND" << endl;
            matched_rules.push_back(r);
        }
    }

    return matched_rules;
}

HandleSeq DefaultForwardChainerCB::get_rootlinks(Handle htarget, AtomSpace* as,
                                                 Type link_type,
                                                 bool subclasses)
{
    auto outgoing = [as](Handle h) {return as->getOutgoing(h);};
    PLNCommons pc(as);
    HandleSeq chosen_roots;
    HandleSeq candidates_roots;
    pc.get_root_links(htarget, candidates_roots);

    for (Handle hr : candidates_roots) {
        bool notexist = find(chosen_roots.begin(), chosen_roots.end(), hr)
                == chosen_roots.end();
        auto type = as->getType(hr);
        bool subtype = (subclasses and classserver().isA(type, link_type));
        if (((type == link_type) or subtype) and notexist) {
            //make sure matches are actually part of the premise list rather than the output of the bindLink
            Handle hpremise = outgoing(outgoing(hr)[1])[0]; //extracting premise from (BindLink((ListLinK..)(ImpLink (premise) (..))))
            if (pc.exists_in(hpremise, htarget)) {
                chosen_roots.push_back(hr);
            }
        }

    }

    return chosen_roots;
}
HandleSeq DefaultForwardChainerCB::choose_input(FCMemory& fcmem)
{
    Handle htarget = fcmem.get_cur_target();
    //get everything associated with the target handle
    HandleSeq inputs = as_->getNeighbors(htarget, true, true, LINK, true);
    return inputs;
}

Handle DefaultForwardChainerCB::choose_next_target(FCMemory& fcmem)
{
    HandleSeq tlist = fcmem.get_target_list();
    map<Handle, float> tournament_elem;
    PLNCommons pc(as_);
    for (Handle t : tlist) {
        float fitness = pc.target_tv_fitness(t);
        tournament_elem[t] = fitness;
    }
    return pc.tournament_select(tournament_elem);
}

//TODO applier should check on atoms (Inference.matched_atoms when Inference.Rule =Cur_Rule), for mutex rules
HandleSeq DefaultForwardChainerCB::apply_rule(FCMemory& fcmem)
{
    Rule * cur_rule = fcmem.get_cur_rule();
    fcpm_->set_fcmem(&fcmem);
    PatternMatch pm;
    try {
        pm.do_bindlink(cur_rule->get_handle(), *fcpm_);
    } catch (InvalidParamException& e) {
        cout << "VALIDATION FAILED:" << endl << e.what() << endl;
    }
    return fcpm_->get_products();
}
