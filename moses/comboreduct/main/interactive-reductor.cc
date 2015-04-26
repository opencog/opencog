/*
 * opencog/comboreduct/main/interactive-reductor.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
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
#include <iostream>
#include <boost/assign/list_of.hpp>

#include <opencog/util/mt19937ar.h>

#include "../reduct/reduct.h"
#include "../reduct/branch_rules.h"
#include "../reduct/meta_rules.h"
#include "../reduct/logical_rules.h"
#include "../reduct/contin_rules.h"
#include "../reduct/general_rules.h"
#include "../reduct/fold_rules.h"
#include "../interpreter/eval.h"
#include "../table/table.h"
#include "../type_checker/type_tree.h"

using namespace std;
using namespace opencog;
using namespace reduct;
using namespace boost::assign;

typedef pair<rule*, string> rule_name_pair;
typedef map<string, rule_name_pair> ref_rule_map;
typedef ref_rule_map::const_iterator ref_rule_map_const_it;
typedef ref_rule_map::iterator ref_rule_map_it;

/**
 * Select the rule given its reference string
 * Returns a null pointer if no valid rule_ref_str or 'h'
 */
const rule* select_rule(string rule_ref_str)
{
    static opencog::combo::vertex_set ignore_ops;
    int reduct_effort = 2;

    const static ref_rule_map ref_rules = 
        map_list_of
        // Logical rules
        ("IA", make_pair((rule*)new downwards(insert_ands()),
                         "insert_ands"))
        ("RUJ", make_pair(new downwards(remove_unary_junctors()),
                          "remove_unary_junctors"))
        ("RDJ", make_pair(new upwards(remove_dangling_junctors()),
                          "remove_dangling_junctors"))
        ("ELI", make_pair(new upwards(eval_logical_identities()),
                          "eval_logical_identities"))
        ("NOT", make_pair(new downwards(reduce_nots()),
                         "reduce_nots"))
        ("OR", make_pair(new downwards(reduce_ors()),
                         "reduce_ors"))
        ("AND", make_pair(new downwards(reduce_ands()),
                         "reduce_ands"))
        ("ENF", make_pair(new subtree_to_enf(),
                          "subtree_to_enf"))

        // Contin buried in below greater_than_zero predicate.
        ("PRD", make_pair(new downwards(simplify_predicates(reduct_effort, ignore_ops)),
                          "simplify_predicates"))
        // Contin rules
        ("PZ", make_pair(new downwards(reduce_plus_zero()),
                          "reduce_plus_zero"))
        ("TOZ", make_pair(new downwards(reduce_times_one_zero()),
                          "reduce_times_one_zero"))
        ("FF", make_pair(new downwards(reduce_factorize_fraction()),
                         "reduce_factorize_fraction"))
        ("F", make_pair(new downwards(reduce_factorize()),
                        "reduce_factorize"))
        ("IC", make_pair(new downwards(reduce_invert_constant()),
                         "reduce_invert_constant"))
        ("FR", make_pair(new downwards(reduce_fraction()),
                         "reduce_fraction"))
        ("TD", make_pair(new downwards(reduce_times_div()),
                         "reduce_times_div"))
        ("PTOC", make_pair(new downwards(reduce_plus_times_one_child()),
                           "reduce_plus_times_one_child"))
        ("SL", make_pair(new downwards(reduce_sum_log()),
                         "reduce_sum_log"))
        ("LDT", make_pair(new downwards(reduce_log_div_times()),
                          "reduce_log_div_times"))
        ("ET", make_pair(new downwards(reduce_exp_times()),
                         "reduce_exp_times"))
        ("ED", make_pair(new downwards(reduce_exp_div()),
                         "reduce_exp_div"))
        ("SIN", make_pair(new downwards(reduce_sin()),
                          "reduce_sin"))

        // Branching and conditional rules
        ("BRA", make_pair(new downwards(reduce_cond_adjacent()),
                          "reduce_cond_adjacent"))
        ("BRC", make_pair(new downwards(reduce_cond_const()),
                          "reduce_cond_const"))
        ("BRE", make_pair(new downwards(reduce_cond_else()),
                          "reduce_cond_else"))
        ("BRP", make_pair(new reduce_cond_arg(reduct_effort, ignore_ops),
                          "reduce_cond_arg"))
        // General rules
        ("LEV", make_pair(new downwards(level()),
                          "level"))
        ("EC", make_pair(new upwards(eval_constants()),
                         "eval_constants"))
        //fold rules
        ("FUR", make_pair(new downwards(fold_unrolling()),
                         "fold_unrolling"));
    if ((rule_ref_str == "h") || (rule_ref_str == "?")) {
        for (ref_rule_map_const_it cit = ref_rules.begin();
            cit != ref_rules.end(); ++cit) {
            cout << cit->first << "\t" << cit->second.second << endl;
        }
        return NULL;
    }
    else {
        ref_rule_map_const_it res = ref_rules.find(rule_ref_str);
        if (res == ref_rules.end()) {
            cout << "Invalid rule (enter h for list of rules)"
                 << endl;
            return NULL;
        }
        else {
            return res->second.first;
        }
    }
}

int main()
{
    randGen().seed(1);

    combo_tree tr;

    string rule_ref_str;

    cout << "Enter the combo to reduce interactively:" << endl;

    while (cin.good()) {
        cin >> tr;
        if (!cin.good())
            break;

        // determine the type signature of tr
        type_tree tt = infer_type_tree(tr);
        cout << "Signature: " << tt << endl;
        
        bool ct = is_well_formed(tt);
        
        if (!ct) {
            cout << "Ill-formed type tree -- Bad type" << endl;
            break;
        }

        while (cin.good()) {
            cout << "Enter a rule to apply on the combo tree "
                 << "(type h for the list of rules): ";

            cin >> rule_ref_str;
            if (!cin.good())
                break;

            //returns a null pointer if no valid rule_ref_str or 'h'
            const rule* selected_rule = select_rule(rule_ref_str);

            if (selected_rule) {

                //produce random inputs
                ITable cti(tt);
                //print cti, for debugging
                //cout << "Rnd matrix :" << endl << cti;
            
                try {
                    //evalutate tr over cti and fill mt1
                    //mixed_table mt1(tr, cti, tr_type);
                    //print mt1, for debugging
                    //cout << "MT1" << endl << mt1 << endl;
                    
                    //print the tree before reduction, for debugging
                    //cout << "Before : " << tr << endl;
                    
                    (*selected_rule)(tr);
                    
                    //evaluate tr over cti and fill mt2
                    //mixed_table mt2(tr, cti, tr_type);
                    //print mt2, for debugging
                    //cout << "MT2" << endl << mt2 << endl;
                    
                    cout << tr << endl;
                    
                    //if (mt1!=mt2) {
                    //    cout << mt1 << endl << mt2 << endl;
                    //    cerr << "mixed-tables don't match!" << endl;
                    //    return 1;
                    //}
                }
                catch(OverflowException& e) {
                    cout << e.get_message() << " : " << e.get_vertex() << endl;
                }
                catch(EvalException& e) {
                    cout << e.get_message() << " : " << e.get_vertex() << endl;
                }
            }
        }
    }
    cout << endl;
    return 0;
}
