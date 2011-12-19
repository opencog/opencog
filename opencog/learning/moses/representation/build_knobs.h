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
#include <boost/ptr_container/ptr_vector.hpp>

#include <opencog/comboreduct/combo/type_tree.h>
#include <opencog/comboreduct/combo/action.h>
#include <opencog/comboreduct/combo/perception.h>

#include "field_set.h"
#include "representation.h"
#include "../moses/using.h"

namespace opencog { namespace moses {

using namespace combo;
using boost::ptr_vector;

// need to call a generator method... (dispatched based on type

// build knobs on a reduced combo tree
struct build_knobs : boost::noncopyable
{
    // used to be ss = 1.0, expansion = 2, depth = 4
    // Optional arguments used only for Petbrain and actions
    build_knobs(RandGen& rng, combo_tree& exemplar,
                const type_tree& tt, representation& rep,
                const operator_set& ignore_ops = operator_set(),
                const combo_tree_ns_set* perceptions = NULL,
                const combo_tree_ns_set* actions = NULL,
                contin_t step_size = 1.0, contin_t expansion = 1.0,
                field_set::arity_t depth = 4);

    void build_logical(combo_tree::iterator it);
    void build_action(combo_tree::iterator it);
    void build_contin(combo_tree::iterator it);
protected:
    RandGen& rng;
    combo_tree& _exemplar;
    const type_tree _type;
    representation& _rep;
    const arity_t _arity; // number of arguments of the combo program
    contin_t _step_size, _expansion;
    field_set::arity_t _depth;

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
    void sample_logical_perms(combo_tree::iterator it,
                              vector<combo_tree>& perms);

    /**
     * @param exemplar reference to the exemplar to apply probe
     * @param it       where in the exemplar
     * @param from     begin iterator of perms (see sample_logical_perms)
     * @param to       end iterator of perms (see sample_logical_perms)
     * @param add_if_in_exemplar add the knob corresponding to perm
     *                           (or negation) if it is already in exemplar
     * @param n_jobs   number of threads to use for the computation
     * @return a ptr_vector of the knobs
     */
    template<typename It>
    ptr_vector<logical_subtree_knob> logical_probe_rec(combo_tree& exemplar,
                                                       combo_tree::iterator it,
                                                       It from, It to,
                                                       bool add_if_in_exemplar,
                                                       unsigned n_jobs = 1) const;
    void logical_cleanup();

    /**
     * Disallow settings of kb that result in a shorter candidate if
     * reduced.
     *
     * Return false if all settings are disallowed, true otherwise.
     */
    bool disc_probe(combo_tree& exemplar, disc_knob_base& kb) const;
    // like the above but exemplar is _exemplar
    bool disc_probe(disc_knob_base& kb);

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
    
    // Assuming that 'it' is '+', then for each variable v, it appends
    // the children *(0 #v). If 'it' is not '+' then '+' is appended
    // as child of 'it' and the same applies to that child. For
    // instance if there are 3 variables and 'it' is a childless '+'
    // then it becomes '+(*(0 #1) *(0 #2) *(0 #3))'
    void append_linear_combination(combo_tree::iterator it);
    
    // is assumes 'it' is '+', appends *(0 v) as child of 'it' and
    // returns the iterator pointing to v
    combo_tree::iterator mult_add(combo_tree::iterator it, const vertex& v);
    void ann_canonize(combo_tree::iterator);

    typedef boost::shared_mutex shared_mutex;
    typedef boost::shared_lock<shared_mutex> shared_lock;
    typedef boost::unique_lock<shared_mutex> unique_lock;

    mutable shared_mutex lp_mutex; // used in logical_probe_thread_safe
    
};

} //~namespace moses
} //~namespace opencog

#endif
