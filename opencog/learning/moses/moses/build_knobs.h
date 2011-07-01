/*
 * opencog/learning/moses/moses/build_knobs.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks, Predrag Janicic
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
#ifndef _MOSES_BUILD_KNOBS_H
#define _MOSES_BUILD_KNOBS_H

#include <boost/utility.hpp>
#include "representation.h"
#include "using.h"
#include <opencog/learning/moses/eda/field_set.h>

#include <opencog/comboreduct/combo/type_tree.h>
#include <opencog/comboreduct/combo/action.h>
#include <opencog/comboreduct/combo/perception.h>

using namespace opencog::combo;

typedef std::set<vertex> operator_set;
typedef operator_set::iterator operator_set_it;

typedef std::set<combo_tree, opencog::size_tree_order<vertex> >
combo_tree_ns_set;
typedef combo_tree_ns_set::iterator combo_tree_ns_set_it;



namespace moses
{

//need to call a generator method... (dispatched based on type

// build knobs on a reduced combo tree
struct build_knobs : boost::noncopyable {
    //used to be ss = 1.0, expansion = 2, depth = 4
    // Optional arguments used only for Petbrain and actions
    build_knobs(opencog::RandGen& rng, combo_tree& exemplar,
                const type_tree& tt, representation& rep,
                const operator_set& ignore_ops = operator_set(),
                const combo_tree_ns_set* perceptions = NULL,
                const combo_tree_ns_set* actions = NULL,
                contin_t step_size = 1.0, contin_t expansion = 1.0,
                eda::field_set::arity_t depth = 4);

    void build_logical(combo_tree::iterator it);
    void build_action(combo_tree::iterator it);
    void build_contin(combo_tree::iterator it);
protected:
    opencog::RandGen& rng;
    combo_tree& _exemplar;
    type_tree _type;
    representation& _rep;
    arity_t _arity; // number of arguments of the combo program
    contin_t _step_size, _expansion;
    eda::field_set::arity_t _depth;

    // perm_ratio ranges from 0 to 1 and defines the number of perms
    // to consider, 0 means _arity positive literals and _arity pairs
    // of literals, 1 means _arity positive literals and
    // _arity*(_arity-1) pairs of literals
    float _perm_ratio;
    const operator_set& _ignore_ops; //the set of operators to ignore
                                     //during representation building
    const combo_tree_ns_set* _perceptions;
    const combo_tree_ns_set* _actions;

    // return true if the following operator is permitted for knob building
    bool permit_ops(const vertex& v);

    void logical_canonize(combo_tree::iterator);
    void add_logical_knobs(combo_tree::iterator it,
                           bool add_if_in_exemplar = true);

    /**
     * fill perms with a set of combinations (subtrees).
     * 
     * For now the combinations (2*_arity) are all positive literals
     * and _arity pairs of j(#i #j) where j is either 'and' or 'or'
     * such that j != *it, #i is a positive literal choosen randomly
     * and #j is a positive or negative literal choosen randomly.
     */
    void sample_logical_perms(combo_tree::iterator it, vector<combo_tree>& perms);
    void logical_probe(const combo_tree& tr, combo_tree::iterator it,
                       bool add_if_in_exemplar);
    void logical_cleanup();
    bool disc_probe(combo_tree::iterator parent, disc_knob_base& kb);

    void action_canonize(combo_tree::iterator);
    void add_action_knobs(combo_tree::iterator it,
                          bool add_if_in_exemplar = true);
    void add_simple_action_knobs(combo_tree::iterator it,
                                 bool add_if_in_exemplar = true);
    void sample_action_perms(combo_tree::iterator it,
                             vector<combo_tree>& perms);
    void simple_action_probe(combo_tree::iterator it, bool add_if_in_exemplar);
    void action_probe(/*const combo_tree& tr*/vector<combo_tree>& perms,
                      combo_tree::iterator it, bool add_if_in_exemplar);
    void action_cleanup();

    void contin_canonize(combo_tree::iterator);
    void canonize_div(combo_tree::iterator it);
    void add_constant_child(combo_tree::iterator it, contin_t v);

    // make it binary * with second arg a constant
    combo_tree::iterator canonize_times(combo_tree::iterator it);

    // call canonize_times on it and then linear_canonize on its first child
    void linear_canonize_times(combo_tree::iterator it);
    void linear_canonize(combo_tree::iterator it);
    void rec_canonize(combo_tree::iterator it);
    void append_linear_combination(combo_tree::iterator it);
    combo_tree::iterator mult_add(combo_tree::iterator it, const vertex& v);
    void ann_canonize(combo_tree::iterator);
};

} //~namespace moses

#endif
