/*
 * opencog/learning/moses/eda/local_structure.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#ifndef _EDA_LOCAL_STRUCTURE_H
#define _EDA_LOCAL_STRUCTURE_H

#include <opencog/util/digraph.h>
#include <opencog/util/oc_assert.h>
#include <opencog/util/tree.h>

#include "../representation/field_set.h"

namespace opencog {
namespace moses {

// Code in this file is used to implement the Bayesian Optimization
// Algorithm (BOA). Given a set of variables, it generates possible
// dependency trees between these variables, and then computes
// conditional probabilities between these variables.  The BOA algorithm
// then attempts to discover the dependency tree that best describes
// the inter-relationships between the variables.
//
// As a special case, the univariate() structure learning function is
// a no-op; that is, it doesn't create any dependency trees.

using std::vector;
using boost::make_counting_iterator;
using boost::make_transform_iterator;


// dtree == dependency tree
//
// a dtree_node can represent 2 things:
//
// 1) if the node is internal then the vector has only one element,
// the idx of the known field involved in the definition of
// conditional probability
//
// 2) if the node is a leaf then the vector size is a + 1 where a is
// the multiplicity of the corresponding field. Each element i in [0, a)
// is the number of "good" instances the (i+1)th disc of the corresponding
// field is encountered. The last element is the sum of all elements
// in [0, a) (useful for computing the probability distribution over
// the elements of the disc)
typedef vector<int> dtree_node;

// decision tree to compute the conditional (or marginal if the tree
// is a single node) probability of each raw disc knowing the value of
// other discs in the field set
typedef tree<dtree_node> dtree;

struct local_structure_model : public nullary_function<instance>,
                               public vector<dtree>
{
    typedef vector<dtree> super; // each entry of dtree corresponds to
                                 // a raw field

    // creates a model based on a set of fields and a range of instances
    template<typename It>
    local_structure_model(const field_set& fields, It from, It to);

    instance operator()() const; // sample from the model

    void split(int, int, dtree::iterator);

protected:
    typedef vector<const instance*> iptr_seq;
    typedef iptr_seq::iterator iptr_iter;

    size_t _instance_length;

    vector<unsigned int> _ordering; // order of field indexes, used
                                    // for model sampling. Each idx in
                                    // the ordering is such that only
                                    // the indexes preceding it are
                                    // possibly used for computing its
                                    // conditional probability

    digraph _initial_deps;      // directed graph representing the
                                // dependencies between fields. This
                                // is solely used to compute _ordering
    field_set _fields;

    // true if the variable at index idx is identical over the range
    // of instances [l, u)
    bool is_uniform_on(iptr_iter l, iptr_iter u, int idx) const;

    void rec_split_term(iptr_iter l, iptr_iter u, int src_idx, int idx,
                        dtree::iterator node, term_tree::iterator osrc);
    void rec_split_contin(iptr_iter l, iptr_iter u,
                          int src_idx, int idx, dtree::iterator node);

    void make_dtree(super::iterator, int);

    void sample(dtree::iterator, disc_t&, const vector<disc_t>&) const;
};

// May be used as a StructureLearningPolicy.
// Simplest case: does no structure learning at all.
// This is suitable for use when all fields/variables in a population
// are known to be statistically independent of one-another.
struct univariate
{
    typedef local_structure_model model_type;

    template<typename It>
    void operator()(const field_set&, It, It,
                    const local_structure_model&) const {}
};

/*
 * XXX implement this. "bde" is "bayesian distribution estimation"
 * I think. 
  struct bde_local_structure_learning {
  typedef local_structure_model model_type;

  //scratch_model should go here
  template<typename It>
  void operator()(const field_set&,It,It,local_structure_model&) const { }
  };*/

// XXX TODO document what this does...
struct local_structure_probs_learning
{
    typedef local_structure_model model_type;

    // Update the model given a field_set and a range of "good"
    // instances.
    template<typename It>
    void operator()(const field_set&, It, It, local_structure_model&) const;
protected:
    // Update the dtree of the model of a field at index idx
    template<typename It>
    void rec_learn(const field_set&, It, It, int, dtree::iterator) const;
};

/////////////TEMPLATIZED FUNCTION IMPLEMENTATION///////////////

// Local structure model constructor.
// Create a model based on a set of variables.
//
// This constructor creates a dependency tree (dtree) for each
// variable (i.e. for each field in the field set).  For contin and
// term variables, we also need to look at each instance as well.
// This is because the value of contin and term variables are trees
// themselves, and we want to create a dtree for each node in each
// instance. So its ot enough to pass just the field_set to this
// constructor, we need to pass in the instances as well.
//
// The iterators are presumed to run over an instance_set<ScoreT>
// The field_set is assumed to describe the fields in the instances.
//
template<typename It>
local_structure_model::local_structure_model(const field_set& fs,
                                             It from, It to) :
    super(fs.raw_size()),
    _instance_length(fs.packed_width()),
    _ordering(make_counting_iterator(0), make_counting_iterator(int(size()))),
    _initial_deps(size()),  // by inheritance, this is vector<dtree>::size()
    _fields(fs)
{
    super::iterator dtr = begin();

    // The dtrees are created in the same order as the variabes appear
    // in the field set.  And these are in the order: terms, contin,
    // disc, bool.  I don't know if its really important to preserve
    // this ordering, but here we are ... To get a better idea of what's
    // going on here, skip to the bottom, and look at disc first, as
    // that is the simplest case.

    // Contin variables and term-valued variables both have trees as
    // values.  We want to create a dtree for each node in each tree.
    // Now, the funny thing here is, the field set is not enough to
    // tell us what possible values we might find in the contin and
    // term variables.  So, to get that, we need to take a look
    // at each field in each instance, pull out the value (which is a
    // tree) and then recusively walk this tree, adding a dtree for
    // each node found. Err, something like that ... 
    if (!_fields.contin().empty() || !_fields.term().empty())
    {
        iptr_seq iptrs(make_transform_iterator(from, addressof<const instance>),
                       make_transform_iterator(to, addressof<const instance>));

        // Iterate over term-algebgra-valued fields
        for (const field_set::term_spec& o : _fields.term())
        {
            int idx_base = distance(begin(), dtr);

            // why + 1?  Because, see note 2) up top. The first n of
            // will hold counts for how often each is observed, and the
            // last is just an accumulator, holding the grand total.
            // We need the grand total later, to compute the fraction.
            // (Do this for disc fields too, down below.)
            make_dtree(dtr++, o.tr->begin().number_of_children() + 1);

            for (field_set::width_t i = 1; i < o.depth; ++i, ++dtr)
            {
                make_dtree(dtr, 0);
                _initial_deps.insert(idx_base + i - 1, idx_base + i);
                // need to recursively split on gggparent, ... , gparent, parent
                rec_split_term(iptrs.begin(), iptrs.end(),
                               idx_base, idx_base + i,
                               dtr->begin(), o.tr->begin());

                //make_dtree(dtr,o.branching);
            }
        }

        // Iterate over all contin-valued fields
        for (const field_set::contin_spec& c : _fields.contin())
        {
            int idx_base = distance(begin(), dtr);
            make_dtree(dtr++, 3); // contin arity is 3=2+1
            for (field_set::width_t i = 1;i < c.depth;++i, ++dtr)
            {
                make_dtree(dtr, 3);
                _initial_deps.insert(idx_base + i - 1, idx_base + i); //add dep to parent

                // recursively split on gggparent, ... , gparent, parent
                /*rec_split_contin(iptrs.begin(),iptrs.end(),
                  idx_base,idx_base+i,dtr->begin());*/

                // Alternatively, only split on parent.
                rec_split_contin(iptrs.begin(), iptrs.end(),
                                 idx_base + i - 1, idx_base + i, dtr->begin());
            }
        }
    }

    // Model discrete & boolean fields.
    // Each discrete variable can take on multiplicity of different
    // values, specifically, one of disc_spec::multy() of them. Thus,
    // each node in the dependency tree will have an arity of multy.
    for (const field_set::disc_spec& d : _fields.disc_and_bit())
        make_dtree(dtr++, d.multy);

    // Now that we have created all of the dtrees, construct a
    // feasible order that respects the initial dependencies
    // XXX ??? Huh? More details, please... 
    randomized_topological_sort(_initial_deps, _ordering.begin());
}

// This presumes that It is an iterator over instance_set<ScoreT>,
// so that from, to define a range of instances (individuals in the
// population).
//
// instance_set is not const so that we can reorder it - the instances
// themselves shouldn't change (XXX why would we want to do this ??)
//
// For univariate(), the dtree for each field will be empty.
//
// The "model" is a set of dtrees, which indicate the dependency between
// each variable in the field set (and more, for contins & terms).  So,
// iterate over the dtrees, and accumulate statistics.
//
// XXX TODO this is unclear, explain what is being accumulated where.
template<typename It>
void local_structure_probs_learning::operator()(const field_set& fs,
                                                It from, It to,
                                                local_structure_model& dst) const
{
    // This is a binary for_each, defined in util/algorithms.h
    for_each(dst.begin(), dst.end(), make_counting_iterator(0),
             bind(&local_structure_probs_learning::rec_learn<It>,
                  this, ref(fs),
                  from, to, _2, bind(&dtree::begin, _1)));
}

// For each node in the dependency tree, accumulate statistics from
// the instances.  Basically, just count how often various values
// occur ...
template<typename It>
void local_structure_probs_learning::rec_learn(const field_set& fs,
                                               It from, It to,
                                               int idx, dtree::iterator dtr) const
{
    // Empty tree: a leaf.
    if (dtr.is_childless())
    {
        while (from != to)
        {
            OC_ASSERT(fs.get_raw(*from, idx) < dtr->size() - 1);
            ++(*dtr)[fs.get_raw(*from++, idx)];
        }
        dtr->back() = accumulate(dtr->begin(), --(dtr->end()), 0);
    }
    else
    { // an internal node (split) - sort [from,to) on the src idx
        int raw_arity = dtr.number_of_children();
        vector<It> pivots(raw_arity + 1);
        pivots.front() = from;
        pivots.back() = to;
        n_way_partition(from, to,
                        bind(&field_set::get_raw, &fs, _1, dtr->front()),
                        raw_arity, ++pivots.begin());
        for_each(pivots.begin(), --pivots.end(), ++pivots.begin(),
                 make_counting_iterator(dtr.begin()),
                 bind(&local_structure_probs_learning::rec_learn<It>, this,
                      ref(fs), _1, _2, idx, _3));
    }
}

} // ~namespace moses
} // ~namespace opencog

#endif
