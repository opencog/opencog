/*
 * opencog/learning/moses/representation/build_knobs.cc
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
#include <future>

#include "build_knobs.h"
#include <opencog/comboreduct/reduct/meta_rules.h>
#include <opencog/comboreduct/reduct/general_rules.h>
#include <opencog/util/iostreamContainer.h>
#include <opencog/util/lazy_random_selector.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/dorepeat.h>
#include <opencog/util/oc_omp.h>

#include <opencog/learning/moses/moses/scoring.h>
#include <opencog/learning/moses/moses/ann_scoring.h>

// uncomment this line for debug information to be given during execution
// #define DEBUG_INFO

namespace opencog { namespace moses {

using namespace std;

typedef combo_tree::sibling_iterator sib_it;
typedef combo_tree::pre_order_iterator pre_it;

build_knobs::build_knobs(combo_tree& exemplar,
                         const combo::type_tree& tt,
                         representation& rep,
                         const operator_set& ignore_ops,
                         const combo_tree_ns_set* perceptions,
                         const combo_tree_ns_set* actions,
                         contin_t step_size,
                         contin_t expansion,
                         field_set::width_t depth)
    : _exemplar(exemplar), _rep(rep),
      _arity(tt.begin().number_of_children() - 1), types(tt),
      _step_size(step_size), _expansion(expansion), _depth(depth),
      _perm_ratio(0),
      _ignore_ops(ignore_ops), _perceptions(perceptions), _actions(actions)
{
    type_tree ot = type_tree_output_type_tree(types);
    type_node output_type = *ot.begin();

    // If there are perceptions/actions, then output had better be
    // an action result.
    if ((perceptions != NULL || actions != NULL) &&
        (output_type != id::action_result_type))
    {
        stringstream ss;
        ss << output_type;
        stringstream art_ss;
        art_ss << id::action_result_type;
        OC_ASSERT(0, "ERROR: During representation building, "
                     "expected action type '%s', got '%s'",
                      art_ss.str().c_str(), ss.str().c_str());
    }

    if (output_type == id::boolean_type) {
        // Exemplar may consist of a mix of boolean and contin-typed
        // variables and functions, but we want the end result to be
        // typed boolean.  Thus, any representation will consist of a
        // tree of logic ops, with anything else contin-valued wrapped
        // up in a predicate (i.e. wrapped by greter_than_zero).
        //
        // Make sure top node of exemplar is a logic op.
        logical_canonize(_exemplar.begin());
        build_logical(_exemplar.begin());
        logical_cleanup();
    }
    else if (output_type == id::contin_type) {
        contin_canonize(_exemplar.begin());
        build_contin(_exemplar.begin());
    }
    else if (output_type == id::enum_type) {
        enum_canonize(_exemplar.begin());
        build_enum(_exemplar.begin());
    }
    else if (output_type == id::action_result_type) {
        // Petbrain
        action_canonize(_exemplar.begin());
        build_action(_exemplar.begin());
        action_cleanup();
    }
    else if (output_type == id::ann_type) {
        // ANN
        ann_canonize(_exemplar.begin());
        build_contin(_exemplar.begin());
    }
    else
    {
        stringstream ss;
        ss << output_type;
        OC_ASSERT(0, "Unsupported output type, got '%s'",
                  ss.str().c_str());

    }
}

/**
 * permitted_op -- return true if the vertex is a permitted operator.
 *
 * An operator is permitted, if it is not one of those that we were
 * told to ignore, i.e. that should be omitted from the tree.
 */
bool build_knobs::permitted_op(const vertex& v)
{
    return _ignore_ops.find(v) == _ignore_ops.end();
}

// ***********************************************************************
// Logic trees: mixture of logic ops, boolean values, and variables.

/**
 * Modify exemplar so that the top node is either a logical_and or a
 * logical_or.
 *
 * @it should point to an expression that is boolean-typed. It may be
 * a boolean-typed argument, or any boolean-valued function (this
 * includes predicates, which are functions that return boolean-typed
 * values).
 *
 * The canonization proceeds recursively if the top node is a predicate.
 * That is, if its a predicate, its argument is canonized as well.
 */
void build_knobs::logical_canonize(pre_it it)
{
    if (*it == id::logical_and) {
        _exemplar.insert_above(it, id::logical_or);
    }
    else if (*it == id::logical_or) {
        _exemplar.insert_above(it, id::logical_and);
    }
    else if (is_boolean(*it)) {
        *it = id::logical_and;
        _exemplar.insert_above(it, id::logical_or);
    }
    else if (is_argument(*it)) {
        it = _exemplar.insert_above(it, id::logical_and);
        _exemplar.insert_above(it, id::logical_or);
    }
    else if (is_predicate(it)) {
        pre_it cit = it;
        if (*it == id::logical_not)   // skip over negation.
            cit = cit.begin();

        // At this time, we assume that all predicates are of type
        // -> (contin boolean).  So, if they take something else, then
        // the linear_canonize will obviously puke.
        linear_canonize(cit.begin());  // get arg to predicate.

        it = _exemplar.insert_above(it, id::logical_and);
        _exemplar.insert_above(it, id::logical_or);
    }
    else {
        OC_ASSERT(0, "Error: during logical_cannonize, got unexpected "
             " type in logical expression.");
    }
}


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
ptr_vector<logical_subtree_knob>
build_knobs::logical_probe_rec(combo_tree& exemplar, pre_it it,
                               It from, It to,
                               bool add_if_in_exemplar,
                               unsigned n_jobs) const
{
    if (n_jobs > 1) {
        auto s_jobs = split_jobs(n_jobs);

        // Define new range
        It mid = from + distance(from, to) / 2;

        // Copy exemplar and it for the second recursive call (this
        // has to be put before the asyncronous call to avoid
        // read/write conflicts)
        combo_tree exemplar_copy(exemplar);
        pre_it it_copy = exemplar_copy.begin();
        advance(it_copy, distance(exemplar.begin(), it));

        // asynchronous recursive call for [from, mid)
        future<ptr_vector<logical_subtree_knob>> f_async =
            async(launch::async,
                  [&]() {return this->logical_probe_rec(exemplar, it, from, mid,
                                                        add_if_in_exemplar,
                                                        s_jobs.first);});

        // synchronous recursive call for [mid, to) on the copy
        ptr_vector<logical_subtree_knob> kb_copy_v =
            logical_probe_rec(exemplar_copy, it_copy, mid, to,
                              add_if_in_exemplar, s_jobs.second);

        // append kb_copy_v to kb_v
        auto kb_v = f_async.get();
        foreach (const logical_subtree_knob& kb_copy, kb_copy_v) {
            kb_v.push_back(new logical_subtree_knob(exemplar, it, kb_copy));
        }
        return kb_v;
    } else {
        ptr_vector<logical_subtree_knob> kb_v;
        while (from != to) {
            auto kb = new logical_subtree_knob(exemplar, it, from->begin());
            if ((add_if_in_exemplar || !kb->in_exemplar())
                && disc_probe(exemplar, *kb))
            {
                kb_v.push_back(kb);
            }
            else
            {
                // gahh. Memory leak if not pushed back!
                delete kb;
            }
            ++from;
        }
        return kb_v;
    }
}

void build_knobs::logical_cleanup()
{
    combo_tree::post_order_iterator it = _exemplar.begin_post();
    while (it != _exemplar.end_post())
        if (is_logical_operator(*it) && it.is_childless())
            _exemplar.erase(it++);
        else
            ++it;
    if (_exemplar.empty())
        _exemplar.set_head(id::logical_and);
}

bool build_knobs::disc_probe(disc_knob_base& kb)
{
    return disc_probe(_exemplar, kb);
}

// XXX This probing and complexity measuring can be rather costly.
// Is it really, really needed?  Can we punt and cross our fingers,
// and live with the few over-expensive cases?  XXX TODO -- measure
// the resulting performance ...
bool build_knobs::disc_probe(combo_tree& exemplar, disc_knob_base& kb) const
{
    using namespace reduct;

    vector<int> to_disallow;

    foreach (int idx, from_one(kb.multiplicity() - 1)) {
        kb.turn(idx);

        /// @todo could use kb.complexity_bound() to be faster, but
        /// there is a strange thing with kb.complexity_bound()
        /// because apparently when it is 0 it actually makes
        /// _exemplar simpler
        complexity_t initial_c = tree_complexity(exemplar);

        // get cleaned and reduced (according to
        // _simplify_knob_building) exemplar
        combo_tree tmp = exemplar; // make a copy -- caution, expensive!
        _rep.clean_combo_tree(tmp, true, true);

        // Note that complexity is negative, with -inf being highest,
        // 0 lowest, which is why this conditional is taken if tmp is
        // simpler after reduction
        if (initial_c < tree_complexity(tmp)) {
            to_disallow.push_back(idx);
        }
    }
    kb.turn(0);

    // if some settings aren't disallowed, make a knob
    if (int(to_disallow.size()) < kb.multiplicity() - 1) {
        foreach (int idx, to_disallow)
            kb.disallow(idx);
        return true;
    } else {
        kb.clear_exemplar();
        return false;
    }
}

// ***********************************************************************
// Predicate (mixed) trees: mixture of logical and predicate terms.

/**
 * Helper function for inserting arguments into a tree of logical
 * operators.  If the argument is of boolean type, then just insert it.
 * If it is of contin type, wrap it up into a predicate, and insert that.
 */
void build_knobs::insert_typed_arg(combo_tree &tr,
                                   type_tree_sib_it arg_type,
                                   argument &arg)
{
    if (*arg_type == id::boolean_type)
    {
        tr.append_child(tr.begin(), arg);
    }
    else if (permitted_op(id::greater_than_zero) &&
         (*arg_type == id::contin_type))
    {
        pre_it gt = tr.append_child(tr.begin(), id::greater_than_zero);
        tr.append_child(gt, arg);
    }
}

/**
 * Create a set of subtrees for the given node.
 *
 * Each subtree must be "permitted", i.e. a legal combination of
 * logical operators, literals and predicates. These are called "perms".
 *
 * Here, a "literal" is one of the input "variables" $i. A positive
 * literal is just $i, and a negative literal is it's negation !$i.
 *
 * For now, there are 2*_arity combinations, consisting of all
 * positive literals, and _arity pairs of op($i $j) where 'op' is one
 * of the logic ops 'and' or 'or', such that op != *it, and $i is a
 * positive literal choosen randomly and $j is a positive or negative
 * literal choosen randomly.
 */
void build_knobs::sample_logical_perms(pre_it it, vector<combo_tree>& perms)
{
    // An argument can be a subtree if it's boolean.
    // If its a contin, then wrap it with "greater_than_zero".
    type_tree_sib_it arg_type = types.begin(types.begin());  // first child
    foreach (int i, from_one(_arity))
    {
        vertex arg = argument(i);
        if (permitted_op(arg))
        {
            if (*arg_type == id::boolean_type)
                perms.push_back(combo_tree(arg));

            else if (permitted_op(id::greater_than_zero) &&
               (*arg_type == id::contin_type))
            {
                combo_tree v(id::greater_than_zero);
                v.append_child(v.begin(), arg);
                perms.push_back(v);
            }
        }
        arg_type++;
    }

    // Also create n random pairs op($i $j) out of the total number
    // 2 * choose(n,2) == n * (n-1) of possible pairs.

    // TODO: should bias the selection of these, so that
    // larger subtrees are preferred .. !? why?
    unsigned int max_pairs = _arity * (_arity - 1);
    if (max_pairs > 0) {
        type_tree_sib_it arg_types = types.begin(types.begin());  // first child
        lazy_random_selector select(max_pairs);

        // Actual number of pairs to create ...
        unsigned int n_pairs =
            _arity + static_cast<unsigned int>(_perm_ratio * (max_pairs - _arity));
        unsigned int count = 0;
        while (count < n_pairs) {
            //while (!select.empty()) {
            combo_tree v(*it == id::logical_and ? id::logical_or : id::logical_and);
            int x = select();
            int a = x / (_arity - 1);
            int b = x - a * (_arity - 1);
            if (b == a)
                b = _arity - 1;

            argument arg_b(1 + b);
            argument arg_a(1 + a);

            // Get the types of arguments a and b.
            // Why doesn't the below just work?
            // type_tree_sib_it type_a = arg_types + a;
            // type_tree_sib_it type_b = arg_types + b;
            type_tree_sib_it type_a = arg_types;
            type_tree_sib_it type_b = arg_types;
            type_a += a;
            type_b += b;

            if (permitted_op(arg_a) && permitted_op(arg_b)) {
                if (b < a) {
                    insert_typed_arg(v, type_b, arg_b);
                    insert_typed_arg(v, type_a, arg_a);

                } else {
                    // As above, but negate arg_a first. So we just
                    // inline-expand insert_typed_arg and put a logical
                    // not in front of it.
                    if (*type_a == id::boolean_type)
                    {
                        arg_a.negate();
                        v.append_child(v.begin(), arg_a);
                    }
                    else if (permitted_op(id::greater_than_zero) &&
                         (*type_a == id::contin_type))
                    {
                        pre_it nt = v.append_child(v.begin(), id::logical_not);
                        pre_it gt = v.append_child(nt, id::greater_than_zero);
                        v.append_child(gt, arg_a);
                    }
                    insert_typed_arg(v, type_b, arg_b);
                }
                perms.push_back(v);
                count ++;
            }
        }
    }

#ifdef DEBUG_INFO
    cerr << "---------------------------------" << endl;
    cerr << endl << "Perms: " << endl;
    foreach (const combo_tree& tr, perms)
        cerr << tr << endl;
    cerr << "---------------------------------" << endl;
#endif
}

/**
 * Add logical and predicate knobs.  The knobs are all boolean valued,
 * and so include knobs for logical operators, knobs for boolean-typed
 * arguments, and knobs for predicates (which are functions that return
 * a boolean value).
 */
void build_knobs::add_logical_knobs(pre_it it, bool add_if_in_exemplar)
{
    // If the node is not a logic op, then bail.  That is, we don't want
    // to insert any kind of boolean knobs into other kinds of ops.
    if (!is_logical_operator(*it))
       return;

    vector<combo_tree> perms;
    sample_logical_perms(it, perms);

    ptr_vector<logical_subtree_knob> kb_v =
        logical_probe_rec(_exemplar, it, perms.begin(), perms.end(),
                          add_if_in_exemplar, num_threads());

    foreach (const logical_subtree_knob& kb, kb_v) {
        _rep.disc.insert(make_pair(kb.spec(), kb));
    }
}

/**
 * build_logical -- add knobs to an exemplar that contains only boolean
 * and logic terms, and no other kinds of terms.
 */
void build_knobs::build_logical(pre_it it)
{
    id::builtin flip = id::null_vertex;

    if (*it == id::logical_not)
    {
        // We allow not's, but only if they preceed a predicate. This
        // is similar to the case with arguments: not's are allowed
        // only if they immediate preceed a boolean-typed argument.
        OC_ASSERT(is_predicate(it.begin()),
              "ERROR: the tree is supposed to be in normal form; "
              "and thus must not contain logical_not nodes.");
    }
    else if (*it == id::logical_and)
    {
        flip = id::logical_or;
    }
    else if (*it == id::logical_or)
    {
        flip = id::logical_and;
    }

    if (flip != id::null_vertex)
    {
        add_logical_knobs(it);
        for (sib_it sib = it.begin(); sib != it.end(); ++sib)
        {
            // Insert logical and/or knobs above arguments and predicates.
            if (is_argument(*sib)) {
                add_logical_knobs(_exemplar.insert_above(sib, flip), false);
            }
            else if (is_predicate(sib)) {
                add_logical_knobs(_exemplar.insert_above(sib, flip), false);

                // At this time, we assume that the only predicate is
                // "greater_than_zero", and it has a single arg, which
                // is either contin or an argument, or any function
                // returning contin ... So, go and insert contin knobs
                // into that expression.
                pre_it pit = sib;
                if (*pit == id::logical_not)  // skip over the not.
                    pit = pit.begin();
                pre_it cit = pit.begin();  // get the arg of predicate.

                OC_ASSERT((is_argument(*cit) || is_contin_expr(*cit)),
                          "Error: predicate term must be made of contin");

                // contin_canonize() creates a big pre-knob expr at @it.
                // build_contin() does the actual knob insertion into
                // the field set.
                contin_canonize(cit);
                build_contin(pit.begin());
            }
            else if (*sib == id::null_vertex)
                break;
            else
                build_logical(sib);
        }
        add_logical_knobs(_exemplar.append_child(it, flip));
    }
}

// ***********************************************************************
// Contin

// Given a pointer @it into the exemplar, the contin_cannonize()
// method will insert a "canonical form" above this pointer.
//
// The canonical form we want for a contin-valued term is a linear-
// weighted fraction whose numerator and denominator are either
// generalized polynomials or other contin-valued terms, i.e.:
//
//    c1 + (c2 * p1) / p2
//
// The generalized polys (p1 and p2) may contain x, sin, abs_log, exp,
// x*y, where x and y are any other contin-valued terms.
//
// We assume that reduction has already taken place, i.e. that p1, p2
// are already in reduced form.
//
// If there are multiple divisors, they will be transformed into separate terms
// (???)
void build_knobs::contin_canonize(pre_it it)
{
    if (is_contin(*it) && get_contin(*it) == 0) {
        *it = id::plus;
    }
    if (*it == id::div) {
        OC_ASSERT((it.number_of_children() == 2),
                  "id::div built in must have exactly 2 children.");
        _exemplar.append_child(_exemplar.insert_above(it, id::plus), contin_t(0));

        canonize_div(it);
    }
    else if (*it == id::plus) {
        // Move the constant child upwards.
        add_constant_child(it, 0);
        _exemplar.insert_above(it, id::plus);
        _exemplar.move_after(it, pre_it(it.last_child()));
        // Handle any divs.
        for (sib_it div = _exemplar.partition(it.begin(), it.end(),
                                              bind(not_equal_to<vertex>(), _1,
                                                   id::div));
             div != it.end();)
            canonize_div(_exemplar.move_after(it, pre_it(div++)));

        // Handle the rest of the children.
        if (permitted_op(id::div)) {
            _exemplar.append_child(_exemplar.insert_above(it, id::div),
                                   contin_t(1));
            canonize_div(_exemplar.parent(it));
        } else
            linear_canonize_times(it);
    }
    else {
        _exemplar.append_child(_exemplar.insert_above(it, id::plus), contin_t(0));
        if (permitted_op(id::div)) {
            _exemplar.append_child(_exemplar.insert_above(it, id::div),
                                   contin_t(1));
            canonize_div(_exemplar.parent(it));
        } else
            linear_canonize_times(it);
    }

#ifdef DEBUG_INFO
    cout << "ok " << _exemplar << endl;
#endif

}

/// build contin -- given a contin expression, insert knobs for that
/// expression into the field set.
///
/// This routine just loops over the children of @it, looking for any
/// contin-valued constants.  These constants are turned into knobs,
/// and added to the field set for the representation.
//
void build_knobs::build_contin(pre_it it)
{
    pre_it end = it;
    end.skip_children();
    for (++end; it != end; ++it) {
        if (is_contin(*it)) {
            // This creates a knob at a particular location in the
            // exemplar inside the representation, and it creates a
            // field spec for the knob. This field spec is not yet
            // yet a part of any field set; this happens later, when
            // the representation is built.
            contin_knob kb(_exemplar, it, _step_size, _expansion, _depth);
            _rep.contin.insert(make_pair(kb.spec(), kb));
        }
    }
}

void build_knobs::canonize_div(pre_it it)
{
    linear_canonize_times(it.begin());
    linear_canonize(it.last_child());
}

void build_knobs::add_constant_child(pre_it it, contin_t v)
{
    sib_it sib = find_if(it.begin(), it.end(), is_contin);
    if (sib == it.end())
        _exemplar.append_child(it, v);
    else
        _exemplar.swap(sib, it.last_child());
}

// Make it binary * with second arg a constant.
pre_it build_knobs::canonize_times(pre_it it)
{
    // get contin child of 'it', if 'it' == 'times' and such contin
    // child exists
    sib_it sib = (*it != id::times ?
                  it.end() : find_if(it.begin(), it.end(), is_contin));
    if (sib == it.end()) {
        pre_it it_times = (*it == id::times?
                           it : _exemplar.insert_above(it, id::times));
        _exemplar.append_child(it_times, contin_t(1));
        return it_times;
    } else if (it.number_of_children() > 2) {
        _exemplar.insert_above(it, id::times);
        _exemplar.move_after(it, pre_it(sib));
        return _exemplar.parent(it);
    } else {
        _exemplar.swap(sib, it.last_child());
        return it;
    }
}

void build_knobs::linear_canonize_times(pre_it it)
{
    linear_canonize(canonize_times(it).begin());
}

/// linear_canonize: insert "pre-knobs" into an exemplar.
/// @it: points into a term of type contin. By convention, this is
///      always pointing into the exemplar.
///
/// This recursively adds "pre-knobs" to all child terms. Such
/// "pre-knobs" are terms that will later be converted into knobs.
/// So, for example, this will insert terms such as *(0 $n) (an arg
/// multiplied by zero), and the zero will later be made a knob.  The
/// "pre-knobs" are always inserted linearly (i.e. prepended by a plus,
/// so that they forma linear combination).
///
/// The recursive aspect of this is such that, if it encounters terms
/// that are functions, it will canonize their arguments as well.  Thus,
/// sin(), exp(), log() will all get their args cannonized.  Note that
/// the function impulse() returns contin but take a boolean expr: in
/// this case, we call logical_canonize on it's arg.
//
void build_knobs::linear_canonize(pre_it it)
{
    // If we're not appending to plus, then insert one.
    if (*it != id::plus)
        it = _exemplar.insert_above(it, id::plus);
    add_constant_child(it, 0);

    // Add the basic elements and recurse, if necessary.
    rec_canonize(it);
}

void build_knobs::rec_canonize(pre_it it)
{
    // cout << "X " << _exemplar << " | " << combo_tree(it) << endl;
    // Recurse on whatever's already present, and create a multiplicand for it.
    if (*it == id::plus) {
        for (sib_it sib = it.begin(); sib != it.end(); ++sib) {
            if (!is_contin(*sib)) {
                sib = canonize_times(sib);
                rec_canonize(sib.begin());
                OC_ASSERT(is_contin(*sib.last_child()),
                          "Sibling's last child isn't id::contin.");
                rec_canonize(_exemplar.insert_above(sib.last_child(), id::plus));
            }
        }

        // Add the basic elements: sin, log, exp, and any variables ($1, ..., $n)
        if (permitted_op(id::sin))
            append_linear_combination(mult_add(it, id::sin));
        if (permitted_op(id::log))
            append_linear_combination(mult_add(it, id::log));
        if (permitted_op(id::exp))
            append_linear_combination(mult_add(it, id::exp));
        append_linear_combination(it);
    }
    // functions that take a single contin arg: canonize the arg.
    else if (*it == id::sin || *it == id::log || *it == id::exp) {
        linear_canonize(it.begin());
    }
    else if ((*it == id::times) || (*it == id::div)) {
        // @todo: think about that case...
        logger().warn("TODO: handle case where it = id::times in build_knobs::rec_canonize");
    }
    // functions that take a single boolean arg: canonize the arg.
    else if (*it == id::impulse) {
        logical_canonize(it.begin());
    }
    else {
        stringstream ss;
        ss << *it << " not a buitin, neither an argument";
        OC_ASSERT(is_argument(*it), ss.str());
    }
}

/// Append a linear combination of *all* arguments. That is, append
/// the term +( *(0 $1) *(0 $2) *(0 $3) ...)  Later on, the zero
/// constants will become knobs.
///
/// The appending happens at location 'it' (which happens to always
/// be in the exampler, in the current usage).  If '*it' isn't plus,
/// then plus is inserted.
///
/// If the argument is of type contin, then it is directly inserted,
/// as shown above.
///
/// If the argument is a boolean, then its run throught the impulse
/// function (1.0 if T and 0.0 if F) and appended with multiplier.
/// That is, its of the form +(... *(0 impulse($n)) ...)
//
void build_knobs::append_linear_combination(pre_it it)
{
    if (*it != id::plus)
        it = _exemplar.append_child(it, id::plus);

    // The type tree holds a lambda which encloses a list of types,
    // i.e. it looks like ->(type type type ... otype)
    // The two begin's skip over the lambda "->"
    type_tree_sib_it tit = types.begin().begin();
    foreach (int idx, from_one(_arity))
    {
        vertex arg = argument(idx);
        if (permitted_op(arg)) {
            if (*tit == id::contin_type) {
                mult_add(it, arg);
            }
            else if (*tit == id::boolean_type) {
                if (permitted_op(id::impulse)) {
                    pre_it imp = mult_add(it, id::impulse);
                    _exemplar.append_child(imp, arg);
                }
            }
            else {
                OC_ASSERT(0,
                    "Error: When building contin expressions, got "
                    "unsupported argument type.");
            }
        }
        tit++;
    }
}

pre_it build_knobs::mult_add(pre_it it, const vertex& v)
{
    pre_it times_it =
        _exemplar.insert_above(_exemplar.append_child(it, contin_t(0)), id::times);
    return _exemplar.append_child(times_it, v);
}

// ***********************************************************************
// Enumerated types.
// For now, we only handle enumerated types on output, and not on input.
// TODO: implement support for enumerated types in the input.

/// enum_cannonize: make sure that the exemplar is in cannonical form.
/// The canonical form will be of the form
///       cond(p1 x1  p2 x2 ... pn xn y)
/// where each possible enum is likely to appear in at least one of the
/// consequents xk (or the else clause y).  Also, each of the predicates pk
/// are also in canonical form.

void build_knobs::enum_canonize(pre_it it)
{
    // If the tree is just a bare enum, put a cond up above it.
    if (is_enum_type(*it)) {
        it = _exemplar.insert_above(it, id::cond);
    }

    // If the node is not a cond, we don't know what to do.
    else if (*it != id::cond)
        return;

    // Append clauses that consist of condition-enum pairs.  Always
    // append at least one clause.  Put them at the back of the cond,
    // the idea being that the clauses at the front of the cond are
    // probably already quite adequate, having been evolved to be good.
    //
    // The inserted logical_and will get blown up into a big rep by the
    // logical_canonize step.

    sib_it last = it.begin();

    // If there is only one clause, don't clobber it.
    // insert always hapens after the iterator, so back up
    // enough to be able to insert.
    size_t sz = _exemplar.number_of_children(it);
    if (1 == sz) {
        last = _exemplar.prepend_child(it, enum_t::get_random_enum());
        _exemplar.prepend_child(it, id::logical_and);
    } else {
        last = _exemplar.child(it, sz - 2);
        _exemplar.insert(last, enum_t::get_random_enum());
        _exemplar.insert(last, id::logical_and);
    }

    // Now that we know where to insert, insert a bunch.
    size_t a_bunch = (2 * enum_t::size()) / 3;
    for (size_t i = 0; i < a_bunch; i++) {
        _exemplar.insert(last, enum_t::get_random_enum());
        _exemplar.insert(last, id::logical_and);
    }

    // Cannonize every predicate. 
    // This loop is strangely structured because the logical_canonize()
    // does an insert_above, and thus wrecks the iterator.
    sib_it sib = it.begin();
    while(1) {
        sib_it next = sib;
        next++;
        if (next == it.end()) break;
        next++;
        if (is_logical_operator(*sib) || is_predicate(sib)) {
            logical_canonize(sib);
        }
        sib = next;
    }
}

// Add knobs.  Right now, none of the enum consequents are knob-able.
// At this point in time, I don't think it matters much (it will
// matter a whole lot more, when enums-in-input-columns are supported).
// If/when this time comes, we'll replace each enum with a disc_knob.
//
// Anyway, what this routine does is to insert knobs for each of the
// predicates that appear in the cond.
//
void build_knobs::build_enum(pre_it it)
{
    // Look for all the predicates, and make sure they get knobs.
    for (sib_it sib = it.begin(); sib != it.end(); ++sib)
    {
        if (is_logical_operator(*sib) || is_predicate(sib)) {
            build_logical(sib);
        }
    }
}

// ***********************************************************************
// Actions for the PetBrain

void build_knobs::action_canonize(pre_it it)
{
    if (*it != id::sequential_and)
        it = _exemplar.insert_above(it, id::sequential_and);
    /*
        if (*it==id::action_success || *it==id::action_failure)
          *it=id::sequential_exec;
        else if (*it!=id::sequential_and && *it!=id::sequential_or)
          it=_exemplar.insert_above(it,id::sequential_and);

        if (*it==id::sequential_and)
          _exemplar.insert_above(it,id::sequential_or);
        else
          _exemplar.insert_above(it,id::sequential_and);*/
}



void build_knobs::build_action(pre_it it)
{

    int p = 80; // probability that controls representational building

    if (*it == id::sequential_and) {

        if (it.is_childless() || randGen().randint(100) < p)
            add_action_knobs(it);

        for (sib_it sib = it.begin(); sib != it.end();++sib) {
            if (is_builtin_action(*sib)) {

                if (randGen().randint(100) > p)
                    add_simple_action_knobs(sib, true);
                else
                    add_action_knobs(sib = _exemplar.insert_above(sib, id::sequential_and), false);
            }
//      else if (*sib==id::null_vertex) break;
            else if (*sib == id::action_boolean_if) {
                if (randGen().randint(100) >= p)
                    add_simple_action_knobs(sib, true);
                build_action(sib);
            }
        }

        if (randGen().randint(100) >= p)
            add_action_knobs(_exemplar.append_child(it, id::sequential_and),
                             false);
    } else if (*it == id::action_boolean_if) {
        for (sib_it sib = ++it.begin(); sib != it.end();++sib)
            if (is_builtin_action(*sib)) {
                add_action_knobs(sib = _exemplar.insert_above(sib, id::sequential_and), false);
            } else if (*sib == id::sequential_and) {
                build_action(sib);
            }
    }
#ifdef DEBUG_INFO
    cout << "=========== current exemplar: " << endl << _exemplar << endl << endl;
#endif
}



void build_knobs::add_action_knobs(pre_it it, bool add_if_in_exemplar)
{
    vector<combo_tree> perms;
    sample_action_perms(it, perms);

    action_probe(perms, it, add_if_in_exemplar);
}


void build_knobs::add_simple_action_knobs(pre_it it, bool add_if_in_exemplar)
{
    simple_action_probe(it, add_if_in_exemplar);
}


void build_knobs::sample_action_perms(pre_it it, vector<combo_tree>& perms)
{
    const int number_of_actions = _actions->size();
    int n = number_of_actions; // controls the number of perms

    for (combo_tree_ns_set_it i = _actions->begin(); i != _actions->end(); ++i)
        perms.push_back(*i);

    // lazy_random_selector sel(number_of_actions);
    // dorepeat(n) {  // add n builtin actions
    //   int i=sel();  // since this argument is varied, it doesn't matter what was the initial value
    // }

    //and n random pairs out of the total  2 * choose(n,2) = n * (n - 1) of these
    //TODO: should bias the selection of these (and possibly choose larger subtrees)
    lazy_random_selector select(number_of_actions*(number_of_actions - 1));

    dorepeat(n) {
        combo_tree v(id::action_boolean_if);
        pre_it iv = v.begin();

        int rand_int = randGen().randint(_perceptions->size());
        combo_tree_ns_set_it p_it = _perceptions->begin();
        for (int ind = 0; ind < rand_int; ++ind)
            ++p_it;

        combo_tree vp(*p_it);
        v.append_child(iv, vp.begin());

        int x = select();
        int a = x / (number_of_actions - 1);
        int b = x - a * (number_of_actions - 1);
        if (b == a)
            b = number_of_actions - 1;

        pre_it ite = (perms[a]).begin();
        v.insert_subtree_after(++iv, ite);
        ite = (perms[b]).begin();
        v.insert_subtree_after(iv, ite);

        perms.push_back(v);
    }
}


void build_knobs::simple_action_probe(pre_it it, bool add_if_in_exemplar)
{
    simple_action_subtree_knob kb(_exemplar, it);
#ifdef DEBUG_INFO
    cout << "simple knob - new exemplar : " << _exemplar << endl;
#endif

    if ((add_if_in_exemplar || !kb.in_exemplar()) /*&& disc_probe(kb) PJ*/)
        _rep.disc.insert(make_pair(kb.spec(), kb));
}


void build_knobs::action_probe(vector<combo_tree>& perms, pre_it it,
                               bool add_if_in_exemplar)
{
    action_subtree_knob kb(_exemplar, it, perms);
#ifdef DEBUG_INFO
    cout << "action knob - new exemplar : " << _exemplar << endl;
#endif

    if ((add_if_in_exemplar || !kb.in_exemplar()) /*&& disc_probe(kb) PJ*/)
        _rep.disc.insert(make_pair(kb.spec(), kb));
}


void build_knobs::action_cleanup()
{
    combo_tree::post_order_iterator it = _exemplar.begin_post();
    while (it != _exemplar.end_post())
        if ((*it == id::sequential_and || *it == id::sequential_and || *it == id::sequential_exec) && it.is_childless())
            _exemplar.erase(it++);
        else
            ++it;
    if (_exemplar.empty())
        _exemplar.set_head(id::sequential_and);
}

// ***********************************************************************
// ANN stuff

#if DEAD_CODE
static int get_max_id(sib_it it, int max_id = 0)
{
    int temp;

    if (!is_ann_type(*it))
        return max_id;

    if ((temp = get_ann_type(*it).idx) > max_id)
        max_id = temp;

    for (sib_it sib = it.begin(); sib != it.end(); ++sib)
        max_id = get_max_id(sib,max_id);

    return max_id;
}
#endif

static void enumerate_nodes(sib_it it, vector<ann_type>& nodes)
{
    if(is_ann_type(*it))
    {
        bool duplicate=false;
        for(vector<ann_type>::iterator node_it = nodes.begin();
                node_it!=nodes.end();++node_it)
            if(get_ann_type(*it).idx==(*node_it).idx)
            {
                duplicate=true;
                break;
            }
        if(!duplicate)
            nodes.push_back(get_ann_type(*it));
    }
    for(sib_it sib=it.begin();sib!=it.end();++sib)
    {
        enumerate_nodes(sib,nodes);
    }
}

void build_knobs::ann_canonize(pre_it it) {
    tree_transform trans;
    cout << _exemplar << endl << endl;
    ann net = trans.decodify_tree(_exemplar);
    cout << &net << endl;
    cout << endl;

    net.add_new_hidden();
    net.add_memory_input();

    cout << &net << endl;
    _exemplar = trans.encode_ann(net);
    cout << _exemplar << endl;

    /*
    cout << "Canonize called..." << endl;
    cout << _exemplar << endl;

    if(get_ann_type(*it).id != id::ann) {
        cout << "root node should be ann" << endl;
    }

    //find maximum id of all nodes in tree
    int max_id = get_max_id(it.begin());
    cout << "MAXID: " << max_id << endl;

    //now create a new node with a larger id
    combo_tree new_node(ann_type(max_id+1,id::ann_node));

    //create connections to this new node from all
    //hidden and input nodes

    //first enumerate all hidden/input nodes
    vector<ann_type> hidden_nodes;
    for (sib_it sib = it.begin(); sib!=it.end(); ++sib) {
        for(sib_it child=sib.begin();child!=sib.end();++child)
         enumerate_nodes(child,hidden_nodes);
    }

    //now create connections in new subtree
    for(vector<ann_type>::iterator node_it = hidden_nodes.begin();
            node_it!=hidden_nodes.end();++node_it)
    {
        new_node.append_child(new_node.begin(),*node_it);
    }

    for(vector<ann_type>::iterator node_it = hidden_nodes.begin();
            node_it!=hidden_nodes.end();++node_it)
    {
        new_node.append_child(new_node.begin(),0.0);
    }

    cout << "Created node: " << new_node << endl;

    //now attach the subtree to the hidden nodes
    //FIXME: now just attaches to the first output
    sib_it first_hidden = it.begin();

    _exemplar.insert_subtree(first_hidden.begin(),new_node.begin());
    _exemplar.insert_after(first_hidden.last_child(),0.0);

    cout << "Completed tree: " << _exemplar << endl;
*/
}

} // ~namespace moses
} // ~namespace opencog
