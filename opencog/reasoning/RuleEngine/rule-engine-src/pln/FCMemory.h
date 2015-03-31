/*
 * FCMemory.h
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>   2015
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
#ifndef FCMEMORY_H_
#define FCMEMORY_H_

#include <opencog/reasoning/RuleEngine/rule-engine-src/Rule.h>
#include <opencog/atomspace/AtomSpace.h>

namespace opencog {

struct Inference {
    int iter_step;
    Rule* applied_rule;
    HandleSeq inf_product;
    HandleSeq matched_nodes; /**<matched nodes with the variables in the rule,useful during mutual exclusion checking*/
};

class ForwardChainer;
class FCMemory {
private:
    friend class ForwardChainer; /*<allow access to private*/
    bool search_in_af_;
    vector<Rule*> rules_; /*<loaded rules*/
    HandleSeq source_list_; /*<selected sources on each forward chaining steps*/
    HandleSeq premise_list_; /*<list of premises*/
    Rule* cur_rule_;
    Handle cur_source_;
    vector<Inference> inf_history_; /*<inference history*/
    AtomSpace* as_;
public:
    FCMemory(AtomSpace* as);
    ~FCMemory();
    vector<Rule*> get_rules();
    void set_rules(vector<Rule*> rules);
    void set_source(Handle source);
    HandleSeq get_source_list();
    HandleSeq get_premise_list();
    void update_premise_list(HandleSeq input);
    bool is_search_in_af();
    Rule* get_cur_rule();
    void add_rules_product(int iteration, HandleSeq product);
    void set_cur_rule(Rule* r);
    void add_inference(int iteration, HandleSeq product,
                       HandleSeq matched_nodes);
    Handle get_cur_source();
    bool isin_source_list(Handle h);
    bool isin_premise_list(Handle h);
    HandleSeq get_result();
    vector<Inference>& get_inf_history();

	// TODO: not used anywhere
	vector<Rule*> get_applied_rules();

};

} // ~namespace opencog

#endif /* FCMEMORY_H_ */
