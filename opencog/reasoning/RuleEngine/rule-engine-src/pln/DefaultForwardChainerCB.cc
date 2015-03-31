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

#include <opencog/atomutils/AtomUtils.h>
#include <opencog/guile/SchemeSmob.h>
#include <opencog/atoms/bind/BindLink.h>

#include "DefaultForwardChainerCB.h"
#include "PLNCommons.h"

using namespace opencog;

DefaultForwardChainerCB::DefaultForwardChainerCB(
        AtomSpace* as, source_selection_mode ts_mode /*=TV_FITNESS_BASED*/) :
        ForwardChainerCallBack(as)
{
    as_ = as;
    fcim_ = new ForwardChainInputMatchCB(as);
    fcpm_ = new ForwardChainPatternMatchCB(as);
    ts_mode_ = ts_mode;
}

DefaultForwardChainerCB::~DefaultForwardChainerCB()
{
    delete fcim_;
    delete fcpm_;
}

/**
 * choose rule based on premises of rule matching the source
 * uses temporary atomspace to limit the search space
 *
 * @param fcmem forward chainer's working memory
 * @return a vector of chosen rules
 */

vector<Rule*> DefaultForwardChainerCB::choose_rule(FCMemory& fcmem)
{
    Handle source = fcmem.get_cur_source();
    if (source == Handle::UNDEFINED or NodeCast(source))
        throw InvalidParamException(TRACE_INFO,
                                    "Needs a source atom of type LINK");
    HandleSeq chosen_bindlinks;

    if (LinkCast(source)) {
        AtomSpace rule_atomspace;
        Handle source_cpy = rule_atomspace.addAtom(source);

        // Copy rules to the temporary atomspace.
        vector<Rule*> rules = fcmem.get_rules();
        for (Rule* r : rules) {
            rule_atomspace.addAtom(r->get_handle());
        }

        // Create bindlink with source as an implicant.
        PLNCommons pc(&rule_atomspace);
        Handle copy = pc.replace_nodes_with_varnode(source_cpy, NODE);
        Handle bind_link = pc.create_bindLink(copy, false);

        // Pattern match.
        BindLinkPtr bl(BindLinkCast(bind_link));
        DefaultImplicator imp(&rule_atomspace);
        imp.implicand = bl->get_implicand();
        imp.set_type_restrictions(bl->get_typemap());
        bl->imply(&imp);

        // Get matched bindLinks.
        HandleSeq matches = imp.result_list;
        if (matches.empty()) {
            logger().debug(
                    "No matching BindLink was found.Returning empty vector");
            return vector<Rule*> { };
        }

        HandleSeq bindlinks;
        for (Handle hm : matches) {
            //get all BindLinks whose part of their premise matches with hm
            HandleSeq hs = get_rootlinks(hm, &rule_atomspace, BIND_LINK);
            for (Handle hi : hs) {
                if (find(bindlinks.begin(), bindlinks.end(), hi) == bindlinks.end()) {
                    bindlinks.push_back(hi);
                }
            }
        }

        // Copy handles to main atomspace.
        for (Handle h : bindlinks) {
            chosen_bindlinks.push_back(as_->addAtom(h));
        }
    }

    // Try to find specialized rules that contain the source node.
    if (NodeCast(source)) {
        chosen_bindlinks = get_rootlinks(source, as_, BIND_LINK);
    }

    // Find the rules containing the bindLink in copied_back.
    vector<Rule*> matched_rules;
    vector<Rule*> rules = fcmem.get_rules();
    for (Rule* r : rules) {
        auto it = find(chosen_bindlinks.begin(), chosen_bindlinks.end(),
                       r->get_handle()); //xxx not matching
        if (it != chosen_bindlinks.end()) {
            matched_rules.push_back(r);
        }
    }

    return matched_rules;
}

/**
 * Gets all top level links of certain types and subclasses that contain @param hsource
 *
 * @param hsource handle whose top level links are to be searched
 * @param as the atomspace in which search is to be done
 * @param link_type the root link types to be searched
 * @param subclasses a flag that tells to look subclasses of @link_type
 */
HandleSeq DefaultForwardChainerCB::get_rootlinks(Handle hsource, AtomSpace* as,
                                                 Type link_type,
                                                 bool subclasses)
{
    auto outgoing = [as](Handle h) {return as->getOutgoing(h);};
    PLNCommons pc(as);
    HandleSeq chosen_roots;
    HandleSeq candidates_roots;
    pc.get_root_links(hsource, candidates_roots);

    for (Handle hr : candidates_roots) {
        bool notexist = find(chosen_roots.begin(), chosen_roots.end(), hr)
                == chosen_roots.end();
        auto type = as->getType(hr);
        bool subtype = (subclasses and classserver().isA(type, link_type));
        if (((type == link_type) or subtype) and notexist) {
            //make sure matches are actually part of the premise list rather than the output of the bindLink
            Handle hpremise = outgoing(outgoing(hr)[1])[0]; //extracting premise from (BindLink((ListLinK..)(ImpLink (premise) (..))))
            if (pc.exists_in(hpremise, hsource)) {
                chosen_roots.push_back(hr);
            }
        }

    }

    return chosen_roots;
}
HandleSeq DefaultForwardChainerCB::choose_premises(FCMemory& fcmem)
{
    HandleSeq inputs;
    PLNCommons pc(as_);
    Handle hsource = fcmem.get_cur_source();

    // Get everything associated with the source handle.
    UnorderedHandleSet neighbors = get_distant_neighbors(hsource, 2);

    // Add all root links of atoms in @param neighbors.
    for (auto hn : neighbors) {
        if (hn->getType() != VARIABLE_NODE) {
            HandleSeq roots;
            pc.get_root_links(hn, roots);
            for (auto r : roots) {
                if (find(inputs.begin(), inputs.end(), r) == inputs.end() and r->getType()
                        != BIND_LINK)
                    inputs.push_back(r);
            }
        }
    }

    return inputs;
}

Handle DefaultForwardChainerCB::choose_next_source(FCMemory& fcmem)
{
    HandleSeq tlist = fcmem.get_premise_list();
    map<Handle, float> tournament_elem;
    PLNCommons pc(as_);
    Handle hchosen = Handle::UNDEFINED;

    for (Handle t : tlist) {
        switch (ts_mode_) {
        case TV_FITNESS_BASED: {
            float fitness = pc.tv_fitness(t);
            tournament_elem[t] = fitness;
        }
            break;
        case STI_BASED:
            tournament_elem[t] = t->getSTI();
            break;
        default:
            throw RuntimeException(TRACE_INFO,
                                   "Unknown source selection mode.");
            break;
        }
    }

    //!Choose a new source that has never been chosen before.
    //!xxx FIXME since same handle might be chosen multiple times the following
    //!code doesn't guarantee all sources have been exhaustively looked.
    for (size_t i = 0; i < tournament_elem.size(); i++) {
        Handle hselected = pc.tournament_select(tournament_elem);
        if (fcmem.isin_source_list(hselected)) {
            continue;
        } else {
            hchosen = hselected;
            break;
        }
    }

    return hchosen;
}

//TODO applier should check on atoms (Inference.matched_atoms when Inference.Rule =Cur_Rule), for mutex rules
HandleSeq DefaultForwardChainerCB::apply_rule(FCMemory& fcmem)
{
    Rule * cur_rule = fcmem.get_cur_rule();
    fcpm_->set_fcmem(&fcmem);

    BindLinkPtr bl(BindLinkCast(cur_rule->get_handle()));
    bl->imply(fcpm_);

    HandleSeq product = fcpm_->get_products();

    //! Make sure the inferences made are new.
    HandleSeq new_product;
    for (auto h : product) {
        if (not fcmem.isin_premise_list(h))
            new_product.push_back(h);
    }

    return new_product;
}
