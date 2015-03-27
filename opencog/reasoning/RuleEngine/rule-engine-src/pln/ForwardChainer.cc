/*
 * ForwardChainer.cc
 *
 * Copyright (C) 2014,2015 Misgana Bayetta
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

#include "ForwardChainer.h"
#include "ForwardChainerCallBack.h"
#include "PLNCommons.h"

#include <opencog/query/PatternMatch.h>
#include <opencog/query/DefaultImplicator.h>
#include <opencog/reasoning/RuleEngine/rule-engine-src/Rule.h>
#include <opencog/guile/SchemeSmob.h>

using namespace opencog;

ForwardChainer::ForwardChainer(AtomSpace * as, string conf_path /*=""*/) :
        as_(as), fcmem_(as_)
{
    if (conf_path != "")
        _conf_path = conf_path;
    init();
}

ForwardChainer::~ForwardChainer()
{
    delete cpolicy_loader_;
}

void ForwardChainer::init()
{
    cpolicy_loader_ = new JsonicControlPolicyParamLoader(as_, _conf_path);
    cpolicy_loader_->load_config();
    fcmem_.search_in_af_ = cpolicy_loader_->get_attention_alloc();
    fcmem_.rules_ = cpolicy_loader_->get_rules();
    fcmem_.cur_rule_ = nullptr;

    // Provide a logger
    log_ = NULL;
    setLogger(new opencog::Logger("forward_chainer.log", Logger::FINE, true));
}

void ForwardChainer::setLogger(Logger* log)
{
    if (log_)
        delete log_;
    log_ = log;
}

Logger* ForwardChainer::getLogger()
{
    return log_;
}

void ForwardChainer::do_chain(ForwardChainerCallBack& fcb,
                              Handle hsource/*=Handle::UNDEFINED*/)
{

    PLNCommons pc(as_);

    //Variable fulfillment query.
    UnorderedHandleSet var_nodes = pc.get_nodes(hsource, { VARIABLE_NODE });
    if (not var_nodes.empty())
        return do_pm(hsource,var_nodes);

    //Forward chaining on a particular type of atom.
    int iteration = 0;
    auto max_iter = cpolicy_loader_->get_max_iter();
    init_source(hsource);
    while (iteration < max_iter /*OR other termination criteria*/) {
        log_->info("Iteration %d", iteration);
        log_->info("Next source %s",
                   SchemeSmob::to_string(fcmem_.cur_source_).c_str());

        //Add more premise to hcurrent_source by pattern matching.
        HandleSeq input = fcb.choose_premises(fcmem_);
        fcmem_.update_premise_list(input);

        //Choose the best rule to apply.
        vector<Rule*> rules = fcb.choose_rule(fcmem_);
        map<Rule*, float> rule_weight;
        for (Rule* r : rules) {
            log_->info("Matching rule %s", r->get_name().c_str());
            rule_weight[r] = r->get_cost();
        }
        auto r = pc.tournament_select(rule_weight);
        log_->info("Chosen rule %s", r->get_name().c_str());

        //!If no rules matches the pattern of the source,choose another source if there is, else end forward chaining.
        if (not r)
            return;
        fcmem_.cur_rule_ = r;

        //!Apply rule.
        log_->info("Applying chosen rule", r->get_name().c_str());
        HandleSeq product = fcb.apply_rule(fcmem_);
        log_->info("Results of rule application");
        for (auto p : product)
            log_->info("%s", SchemeSmob::to_string(p).c_str());
        fcmem_.add_rules_product(iteration, product);
        fcmem_.update_premise_list(product);

        //!Choose next source.
        auto source = fcb.choose_next_source(fcmem_);
        fcmem_.set_source(source);
        iteration++;
    }
}

/**
 * Does pattern matching for a variable containing query.
 * @param source handle containing VariableNode
 */
void ForwardChainer::do_pm(const Handle& hsource,const UnorderedHandleSet& var_nodes)
{
    DefaultImplicator impl(as_);
    impl.implicand = hsource;
    PatternMatch pm;
    PatternMatchEngine pme;
    HandleSeq vars;
    for (auto h : var_nodes)
        vars.push_back(h);
    Handle hvar_list = as_->addLink(LIST_LINK, vars);
    Handle hclause = as_->addLink(AND_LINK, hsource);
    pm.match(&impl, hvar_list, hclause);

    //update result
    fcmem_.add_rules_product(0, impl.result_list);

    //Delete the AND_LINK and LIST_LINK
    as_->removeAtom(hvar_list);
    as_->removeAtom(hclause);

    return;
}
void ForwardChainer::init_source(Handle hsource)
{
    if (hsource == Handle::UNDEFINED) {
        log_->info("Choosing a random source");
        fcmem_.set_source(choose_random_source(as_)); //start FC on a random source
    } else {
        fcmem_.set_source(hsource);
    }
}

Handle ForwardChainer::choose_random_source(AtomSpace * as)
{
    //!choose a random atoms to start forward chaining with
    HandleSeq hs;
    if (cpolicy_loader_->get_attention_alloc())
        as->getHandleSetInAttentionalFocus(back_inserter(hs));
    else
        as->getHandlesByType(back_inserter(hs), ATOM, true);
    Handle rand_source;
    for (;;) {
        Handle h = hs[rand() % hs.size()];
        Type t = as->getType(h);
        if (t != VARIABLE_NODE and t != BIND_LINK and t != IMPLICATION_LINK) {
            rand_source = h;
            break;
        }
    }
    return rand_source;
}

HandleSeq ForwardChainer::get_chaining_result()
{
    return fcmem_.get_result();
}
