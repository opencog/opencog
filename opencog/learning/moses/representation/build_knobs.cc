/*
 * opencog/learning/moses/moses/build_knobs.cc
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

build_knobs::build_knobs(RandGen& _rng,
                         combo_tree& exemplar,
                         const combo::type_tree& tt,
                         representation& rep,
                         const operator_set& ignore_ops,
                         const combo_tree_ns_set* perceptions,
                         const combo_tree_ns_set* actions,
                         contin_t step_size,
                         contin_t expansion,
                         eda::field_set::arity_t depth)
    : rng(_rng), _exemplar(exemplar), _type(tt), _rep(rep),
      _arity(tt.begin().number_of_children() - 1),
      _step_size(step_size), _expansion(expansion), _depth(depth),
      _perm_ratio(0),
      _ignore_ops(ignore_ops), _perceptions(perceptions), _actions(actions)
{
    type_tree output_type = combo::type_tree_output_type_tree(_type);
    combo::type_tree action_result_type_tree =
        combo::type_tree(combo::id::action_result_type);
    combo::type_tree boolean_type_tree =
        combo::type_tree(combo::id::boolean_type);
    combo::type_tree contin_type_tree =
        combo::type_tree(combo::id::contin_type);
    combo::type_tree ann_type_tree =
        combo::type_tree(combo::id::ann_type);
    // OC_ASSERT
    stringstream ss;
    ss << output_type;
    stringstream art_ss; //action_result_type
    art_ss << combo::id::action_result_type;
    OC_ASSERT((((perceptions != NULL || actions != NULL) && 
                output_type == action_result_type_tree) ||
               output_type == boolean_type_tree ||
               output_type == contin_type_tree ||
               output_type == ann_type_tree),
              "Types differ. Expected '%s', got '%s'",
              art_ss.str().c_str(), ss.str().c_str());
    // ~OC_ASSERT
    if (output_type == boolean_type_tree) {
        logical_canonize(_exemplar.begin());
        build_logical(_exemplar.begin());
        logical_cleanup();
    } else if (output_type == action_result_type_tree) {
        // Petbrain
        action_canonize(_exemplar.begin());
        build_action(_exemplar.begin());
        action_cleanup();
    } else if (output_type == ann_type_tree) {
        // ANN
        ann_canonize(_exemplar.begin());
        build_contin(_exemplar.begin());
    } else {
        OC_ASSERT(output_type == contin_type_tree,
                  "Types differ. Expected 'combo::id::contin_type', got '%s'",
                  ss.str().c_str());

        contin_canonize(_exemplar.begin());
        build_contin(_exemplar.begin());
    }
}



void build_knobs::build_logical(pre_it it)
{
    OC_ASSERT(*it != id::logical_not,
              "the tree is supposed to be in normal form thus have no logical_not");

    add_logical_knobs(it);

    if (*it == id::logical_and) {
        for (combo_tree::sibling_iterator sib = it.begin();
             sib != it.end();++sib)
            if (is_argument(*sib))
                add_logical_knobs(_exemplar.insert_above(sib, id::logical_or),
                                  false);
            else if (*sib == id::null_vertex)
                break;
            else
                build_logical(sib);
        add_logical_knobs(_exemplar.append_child(it, id::logical_or));
    } else if (*it == id::logical_or) {
        for (combo_tree::sibling_iterator sib = it.begin();
             sib != it.end();++sib)
            if (is_argument(*sib))
                add_logical_knobs(_exemplar.insert_above(sib, id::logical_and),
                                  false);
            else if (*sib == id::null_vertex)
                break;
            else
                build_logical(sib);
        add_logical_knobs(_exemplar.append_child(it, id::logical_and));
    }
}

bool build_knobs::permit_ops(const vertex& v)
{
    return _ignore_ops.find(v) == _ignore_ops.end();
}

void build_knobs::logical_canonize(pre_it it)
{
    if(is_boolean(*it))
        *it = id::logical_and;
    else if (*it != id::logical_and && *it != id::logical_or)
        it = _exemplar.insert_above(it, id::logical_and);

    if (*it == id::logical_and)
        _exemplar.insert_above(it, id::logical_or);
    else
        _exemplar.insert_above(it, id::logical_and);
}


void build_knobs::add_logical_knobs(pre_it it, bool add_if_in_exemplar)
{
    vector<combo_tree> perms;
    sample_logical_perms(it, perms);

    ptr_vector<logical_subtree_knob> kb_v =
        logical_probe_rec(_exemplar, it, perms.begin(), perms.end(),
                          add_if_in_exemplar, num_threads());
    foreach(const logical_subtree_knob& kb, kb_v)
        _rep.disc.insert(make_pair(kb.spec(), kb));
}

void build_knobs::sample_logical_perms(pre_it it, vector<combo_tree>& perms)
{
    //all n literals
    foreach(int i, from_one(_arity)) {
        vertex arg = argument(i);
        if(permit_ops(arg))
            perms.push_back(combo_tree(arg));
    }

    // and n random pairs out of the total
    // 2 * choose(n,2) = n * (n - 1) of these

    // TODO: should bias the selection of these (and possibly choose
    // larger subtrees)
    unsigned int max_pairs = _arity*(_arity - 1);
    if(max_pairs > 0) {
        lazy_random_selector select(max_pairs, rng);
        unsigned int n_pairs =
            _arity + static_cast<unsigned int>(_perm_ratio * (max_pairs - _arity));
        dorepeat(n_pairs) {
            //while (!select.empty()) {
            combo_tree v(*it == id::logical_and ? id::logical_or : id::logical_and);
            int x = select();
            int a = x / (_arity - 1);
            int b = x - a * (_arity - 1);
            if (b == a)
                b = _arity - 1;
            
            argument arg_b(1 + b);
            argument arg_a(1 + a);
            
            if(permit_ops(arg_a) && permit_ops(arg_b)) {
                if (b < a) {
                    v.append_child(v.begin(), arg_b);
                    v.append_child(v.begin(), arg_a);
                } else {
                    arg_a.negate();
                    v.append_child(v.begin(), arg_a);
                    v.append_child(v.begin(), arg_b);
                }
                perms.push_back(v);
            }
        }
    }

#ifdef DEBUG_INFO
    cerr << "---------------------------------" << endl;
    cerr << endl << "Perms: " << endl;
    foreach(const combo_tree& tr, perms)
        cerr << tr << endl;
    cerr << "---------------------------------" << endl;
#endif
}

template<typename It>
ptr_vector<logical_subtree_knob>
build_knobs::logical_probe_rec(combo_tree& exemplar, pre_it it,
                               It from, It to,
                               bool add_if_in_exemplar,
                               unsigned n_jobs) const {
    if(n_jobs > 1) {
        auto s_jobs = split_jobs(n_jobs);

        // define new range
        It mid = from + distance(from, to) / 2;

        // copy exemplar and it for the second recursive call (this
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
        foreach(const logical_subtree_knob& kb_copy, kb_copy_v) {
            kb_v.push_back(new logical_subtree_knob(exemplar, it, kb_copy));
        }
        return kb_v;
    } else {
        ptr_vector<logical_subtree_knob> kb_v;
        while(from != to) {
            auto kb = new logical_subtree_knob(exemplar, it, from->begin());
            if ((add_if_in_exemplar || !kb->in_exemplar())
                && disc_probe(exemplar, *kb)) {
                kb_v.push_back(kb);
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
        if(is_logical_operator(*it) && it.is_childless())
            _exemplar.erase(it++);
        else
            ++it;
    if (_exemplar.empty())
        _exemplar.set_head(id::logical_and);
}

bool build_knobs::disc_probe(disc_knob_base& kb) {
    return disc_probe(_exemplar, kb);
}

bool build_knobs::disc_probe(combo_tree& exemplar, disc_knob_base& kb) const
{
    using namespace reduct;

    vector<int> to_disallow;

    foreach(int idx, from_one(kb.arity() - 1)) {
        kb.turn(idx);

        /// @todo could use kb.complexity_bound() to be faster, but
        /// there is a strange thing with kb.complexity_bound()
        /// because apparently when it is 0 it actually makes
        /// _exemplar simpler
        complexity_t initial_c = complexity(exemplar);

        // get cleaned and reduced (according to
        // _simplify_knob_building) exemplar
        combo_tree tmp = _rep.get_clean_combo_tree(exemplar, true, true);

        // Note that complexity is negative, with -inf being highest,
        // 0 lowest, which is why this conditional is taken if tmp is
        // simpler after reduction
        if (initial_c < complexity(tmp)) {
            to_disallow.push_back(idx);
        }
    }
    kb.turn(0);

    // if some settings aren't disallowed, make a knob
    if (int(to_disallow.size()) < kb.arity() - 1) {
        foreach (int idx, to_disallow)
            kb.disallow(idx);
        return true;
    } else {
        kb.clear_exemplar();
        return false;
    }
}



// ***********************************************************************
// Actions

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

        if (it.is_childless() || rng.randint(100) < p)
            add_action_knobs(it);

        for (combo_tree::sibling_iterator sib = it.begin();
             sib != it.end();++sib) {
            if (is_builtin_action(*sib)) {

                if (rng.randint(100) > p)
                    add_simple_action_knobs(sib, true);
                else
                    add_action_knobs(sib = _exemplar.insert_above(sib, id::sequential_and), false);
            }
//      else if (*sib==id::null_vertex) break;
            else if (*sib == id::action_boolean_if) {
                if (rng.randint(100) >= p)
                    add_simple_action_knobs(sib, true);
                build_action(sib);
            }
        }

        if (rng.randint(100) >= p)
            add_action_knobs(_exemplar.append_child(it, id::sequential_and),
                             false);
    } else if (*it == id::action_boolean_if) {
        for (combo_tree::sibling_iterator sib = ++it.begin();
             sib != it.end();++sib)
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
    lazy_random_selector select(number_of_actions*(number_of_actions - 1), rng);

    dorepeat(n) {
        combo_tree v(id::action_boolean_if);
        pre_it iv = v.begin();

        int rand_int = rng.randint(_perceptions->size());
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
// Contin

// the canonical form we want for a term is linear-weighted fraction whose
// numerator & denominator are either generalized polynomials or terms, i.e.:
//
// c1 + (c2 * p1) / p2
//
// generalized polys (p1 and p2) can contain:
//
// x, sin, abs_log, exp, x*y
// (where x & y are any vars)
//
// we assume that reduction has already taken place
//
// if there are multiple divisors, they will be transformed into separate terms
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
    } else if (*it == id::plus) {
        //move the constant child upwards
        add_constant_child(it, 0);
        _exemplar.insert_above(it, id::plus);
        _exemplar.move_after(it, pre_it(it.last_child()));
        //handle any divs
        for (sib_it div = _exemplar.partition(it.begin(), it.end(),
                                              bind(not_equal_to<vertex>(), _1,
                                                   id::div));
             div != it.end();)
            canonize_div(_exemplar.move_after(it, pre_it(div++)));
        //handle the rest of the children

        if(permit_ops(id::div)) {
            _exemplar.append_child(_exemplar.insert_above(it, id::div),
                                   contin_t(1));
            canonize_div(_exemplar.parent(it));
        } else 
            linear_canonize_times(it);
    } else {
        _exemplar.append_child(_exemplar.insert_above(it, id::plus), contin_t(0));
        if(permit_ops(id::div)) {
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


void build_knobs::build_contin(pre_it it)
{
    pre_it end = it;
    end.skip_children();
    for (++end;it != end;++it)
        if (is_contin(*it)) {
            contin_knob kb(_exemplar, it, _step_size, _expansion, _depth);
            _rep.contin.insert(make_pair(kb.spec(), kb));
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

//make it binary * with second arg a constant
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

void build_knobs::linear_canonize(pre_it it)
{
    //make it a plus
    if (*it != id::plus)
        it = _exemplar.insert_above(it, id::plus);
    add_constant_child(it, 0);

    //add the basic elements and recurse, if necessary
    rec_canonize(it);
}

void build_knobs::rec_canonize(pre_it it)
{
    // cout << "X " << _exemplar << " | " << combo_tree(it) << endl;
    //recurse on whatever's already present, and create a multiplicand for it
    if (*it == id::plus) {
        for (sib_it sib = it.begin();sib != it.end();++sib) {
            if (!is_contin(*sib)) {
                sib = canonize_times(sib);
                rec_canonize(sib.begin());
                OC_ASSERT(is_contin(*sib.last_child()),
                          "Sibling's last child isn't id::contin.");
                rec_canonize(_exemplar.insert_above(sib.last_child(), id::plus));
            }
        }
        //add the basic elements: sin, log, exp, and any variables (#1, ..., #n)
        if(permit_ops(id::sin))
            append_linear_combination(mult_add(it, id::sin));
        if(permit_ops(id::log))
            append_linear_combination(mult_add(it, id::log));
        if(permit_ops(id::exp))
            append_linear_combination(mult_add(it, id::exp));
        append_linear_combination(it);
    } else if (*it == id::sin || *it == id::log || *it == id::exp) {
        cout << _exemplar << " | " << combo_tree(it) << endl;
        linear_canonize(it.begin());
    } else if (*it == id::times) {
        // @todo: think about that case...
        cerr << "I (Nil) must think for that one" << endl;
    } else {
        stringstream ss;
        ss << *it << " not a buitin, neither an argument";
        OC_ASSERT(is_argument(*it), ss.str());
    }
}

void build_knobs::append_linear_combination(pre_it it)
{
    if (*it != id::plus)
        it = _exemplar.append_child(it, id::plus);
    foreach(int idx, from_one(_arity)) {
        vertex arg = argument(idx);
        if(permit_ops(arg))
            mult_add(it, arg);
    }
}

pre_it build_knobs::mult_add(pre_it it, const vertex& v)
{
    pre_it times_it = _exemplar.insert_above(_exemplar.append_child(it,
                                                                    contin_t(0)),
                                             id::times);
    return --_exemplar.append_child(times_it, v);
}

static int get_max_id(sib_it it, int max_id = 0)
{
    int temp;
    
    if(!is_ann_type(*it))
        return max_id;

    if((temp=get_ann_type(*it).idx)>max_id)
        max_id=temp;
    
    for (sib_it sib = it.begin(); sib!=it.end(); ++sib) 
        max_id=get_max_id(sib,max_id);
    
    return max_id;
}

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
