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

struct Inference {
    int iter_step;
    Rule* applied_rule;
    HandleSeq inf_product;
    HandleSeq matched_nodes; /**<matched nodes with the variables in the rule,useful during mutual exclusion checking*/
};

using namespace opencog;

class ForwardChainer;
class FCMemory {
private:
    friend class ForwardChainer; /*<allow access to private*/
    bool search_in_af_;
    vector<Rule*> rules_; /*<loaded rules*/
    HandleSeq target_list_; /*<selected targets on each forward chaining steps*/
    HandleSeq premise_list_; /*<list of premises*/
    Rule* cur_rule_;
    Handle cur_target_;
    vector<Inference> inf_history_; /*<inference history*/
    AtomSpace* as_;
    void update_premise_list(HandleSeq input);
public:
    FCMemory(AtomSpace* as);
    ~FCMemory();
    vector<Rule*> get_rules(void);
    void set_rules(vector<Rule*> rules);
    void set_target(Handle target);
    HandleSeq get_target_list(void);
    HandleSeq get_premise_list(void);
    bool is_search_in_af(void);
    Rule* get_cur_rule(void);
    void add_rules_product(int iteration, HandleSeq product);
    void set_cur_rule(Rule* r);
    void add_inference(int iteration, HandleSeq product,
                       HandleSeq matched_nodes);
    Handle get_cur_target(void);bool isin_premise_list(Handle h);
    HandleSeq get_result(void);
    vector<Inference>& get_inf_history(void);
    vector<Rule*> get_applied_rules(void);

};

#endif /* FCMEMORY_H_ */
