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

#include <opencog/util/Logger.h>
#include <opencog/atoms/bind/SatisfactionLink.h>
#include <opencog/atomutils/AtomUtils.h>
#include <opencog/query/DefaultImplicator.h>
#include <opencog/reasoning/RuleEngine/rule-engine-src/Rule.h>
#include <opencog/atoms/bind/BindLink.h>
#include "ForwardChainer.h"
#include "ForwardChainerCallBack.h"
#include "PLNCommons.h"

using namespace opencog;

ForwardChainer::ForwardChainer(AtomSpace * as, string conf_path /*=""*/) :
        _as(as), _fcmem(_as)
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
    cpolicy_loader_ = new JsonicControlPolicyParamLoader(_as, _conf_path);
    cpolicy_loader_->load_config();
    _fcmem.search_in_af_ = cpolicy_loader_->get_attention_alloc();
    _fcmem.rules_ = cpolicy_loader_->get_rules();
    _fcmem.cur_rule_ = nullptr;

    // Provide a logger
    _log = NULL;
    setLogger(new opencog::Logger("forward_chainer.log", Logger::FINE, true));
}

void ForwardChainer::setLogger(Logger* log)
{
    if (_log)
        delete _log;
    _log = log;
}

Logger* ForwardChainer::getLogger()
{
    return _log;
}

/**
 * Does one step forward chaining
 * @return false if there is not target to explore
 */
bool ForwardChainer::step(ForwardChainerCallBack& fcb)
{
    PLNCommons pc(_as); //utility class

    if (_fcmem.get_cur_source() == Handle::UNDEFINED) {
        _log->info(
                "[ForwardChainer] No current source, forward chaining aborted");
        return false;
    }

    _log->info("[ForwardChainer] Next source %s",
               _fcmem.cur_source_->toString().c_str());

    // Add more premise to hcurrent_source by pattern matching.
    _log->info("[ForwardChainer] Choose additional premises:");
    HandleSeq input = fcb.choose_premises(_fcmem);
    for (Handle h : input) {
        if (not _fcmem.isin_premise_list(h))
            _log->info("%s \n", h->toString().c_str());
    }
    _fcmem.update_premise_list(input);

    // Choose the best rule to apply.
    vector<Rule*> rules = fcb.choose_rules(_fcmem);
    map<Rule*, float> rule_weight;
    for (Rule* r : rules) {
        _log->info("[ForwardChainer] Matching rule %s", r->get_name().c_str());
        rule_weight[r] = r->get_cost();
    }
    auto r = pc.tournament_select(rule_weight);

    //! If no rules matches the pattern of the source, choose
    //! another source if there is, else end forward chaining.
    if (not r) {
        auto new_source = fcb.choose_next_source(_fcmem);
        if (new_source == Handle::UNDEFINED) {
            _log->info(
                    "[ForwardChainer] No chosen rule and no more target to choose.Aborting forward chaining.");
            return false;
        } else {
            _log->info(
                    "[ForwardChainer] No matching rule,attempting with another target %s.",
                    new_source->toString().c_str());
            //set source and try another step
            _fcmem.set_source(new_source);
            return step(fcb);
        }
    }

    _fcmem.cur_rule_ = r;

    //! Apply rule.
    _log->info("[ForwardChainer] Applying chosen rule %s",
               r->get_name().c_str());
    HandleSeq product = fcb.apply_rule(_fcmem);

    _log->info("[ForwardChainer] Results of rule application");
    for (auto p : product)
        _log->info("%s", p->toString().c_str());
    _log->info("[ForwardChainer] adding inference to history");
    _fcmem.add_rules_product(iteration, product);
    _log->info(
            "[ForwardChainer] updating premise list with the inference made");
    _fcmem.update_premise_list(product);

    return true;
}

void ForwardChainer::do_chain(ForwardChainerCallBack& fcb,
                              Handle hsource/*=Handle::UNDEFINED*/)
{
    if (hsource == Handle::UNDEFINED) {
        do_pm();
        return;
    }
    // Variable fulfillment query.
    UnorderedHandleSet var_nodes = get_outgoing_nodes(hsource,
                                                      { VARIABLE_NODE });
    if (not var_nodes.empty())
        return do_pm(hsource, var_nodes, fcb);

    auto max_iter = _cpolicy_loader->get_max_iter();
    while (_iteration < max_iter /*OR other termination criteria*/) {

        _log->info("Iteration %d", _iteration);
        if (_iteration == 0)
            _fcmem.set_source(hsource);

        if (not step(fcb))
            break;

        //! Choose next source.
        _log->info("[ForwardChainer] setting next source");
        _fcmem.set_source(fcb.choose_next_source(_fcmem));

        _iteration++;
    }
}

/**
 * Does pattern matching for a variable containing query.
 * @param source a variable containing handle passed as an input to the pattern matcher
 * @param var_nodes the VariableNodes in @param hsource
 * @param fcb a forward chainer callback implementation used here only for choosing rules
 * that contain @param hsource in their implicant
 */
void ForwardChainer::do_pm(const Handle& hsource,
                           const UnorderedHandleSet& var_nodes,
                           ForwardChainerCallBack& fcb)
{
    DefaultImplicator impl(_as);
    impl.implicand = hsource;
    HandleSeq vars;
    for (auto h : var_nodes)
        vars.push_back(h);
    _fcmem.set_source(hsource);
    Handle hvar_list = _as->addLink(VARIABLE_LIST, vars);
    Handle hclause = _as->addLink(AND_LINK, hsource);

    // Run the pattern matcher, find all patterns that satisfy the
    // the clause, with the given variables in it.
    SatisfactionLinkPtr sl(createSatisfactionLink(hvar_list, hclause));
    sl->satisfy(impl);

    // Update result
    _fcmem.add_rules_product(0, impl.result_list);

    // Delete the AND_LINK and LIST_LINK
    _as->removeAtom(hvar_list);
    _as->removeAtom(hclause);

    //!Additionally, find applicable rules and apply.
    vector<Rule*> rules = fcb.choose_rules(_fcmem);
    for (Rule* rule : rules) {
        BindLinkPtr bl(BindLinkCast(rule->get_handle()));
        DefaultImplicator impl(_as);
        impl.implicand = bl->get_implicand();
        bl->imply(impl);
        _fcmem.set_cur_rule(rule);
        _fcmem.add_rules_product(0, impl.result_list);
    }
}
/**
 * Invokes pattern matcher using each rule declared in the configuration file.
 */
void ForwardChainer::do_pm()
{
    //! Do pattern matching using the rules declared in the declaration file
    _log->info(
            "Forward chaining on the entire atomspace with rules declared in %s",
            _conf_path.c_str());
    vector<Rule*> rules = _fcmem.get_rules();
    for (Rule* rule : rules) {
        _log->info("Applying rule %s on ", rule->get_name().c_str());
        BindLinkPtr bl(BindLinkCast(rule->get_handle()));
        DefaultImplicator impl(_as);
        impl.implicand = bl->get_implicand();
        bl->imply(impl);
        _fcmem.set_cur_rule(rule);

        _log->info("OUTPUTS");
        for (auto h : impl.result_list)
            _log->info("%s", h->toString().c_str());

        _fcmem.add_rules_product(0, impl.result_list);
    }

}

HandleSeq ForwardChainer::get_chaining_result()
{
    return _fcmem.get_result();
}
