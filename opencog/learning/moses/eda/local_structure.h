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

#include "../representation/field_set.h"

namespace opencog { 
namespace moses {

using std::vector;

typedef vector<int> dtree_node;
typedef tree<dtree_node> dtree;

struct local_structure_model : public nullary_function<instance>,
                               public vector<dtree> {
    typedef vector<dtree> super;

    //creates a model based on a set of fields and a range of instances
    template<typename It>
    local_structure_model(const field_set& fields,
                          It from, It to, RandGen& _rng);

    instance operator()() const; //sample from the model

    void split(int,int, dtree::iterator);

protected:
    typedef vector<const instance*> iptr_seq;
    typedef iptr_seq::iterator iptr_iter;

    int _instance_length;
    vector<unsigned int> _ordering;
    digraph _initial_deps;
    field_set _fields;
    RandGen& rng;

    //true if the range is uniform on the variable at index idx
    bool is_uniform_on(iptr_iter l, iptr_iter u, int idx);

    void rec_split_onto(iptr_iter l, iptr_iter u, int src_idx, int idx,
                        dtree::iterator node, onto_tree::iterator osrc);
    void rec_split_contin(iptr_iter l, iptr_iter u,
                          int src_idx, int idx, dtree::iterator node);

    void make_dtree(super::iterator, int);

    void sample(dtree::iterator, disc_t&, const vector<disc_t>&) const;
};

struct univariate { //i.e., no structure learning
    typedef local_structure_model model_type;

    template<typename It>
    void operator()(const field_set&, It, It,
                    const local_structure_model&) const { }
};

/*struct bde_local_structure_learning {
  typedef local_structure_model model_type;

  //scratch_model should go here
  template<typename It>
  void operator()(const field_set&,It,It,local_structure_model&) const { }
  };*/

struct local_structure_probs_learning {
    typedef local_structure_model model_type;

    template<typename It>
    void operator()(const field_set&, It, It, local_structure_model&) const;
protected:
    template<typename It>
    void rec_learn(const field_set&, It, It, int, dtree::iterator) const;
};

/////////////TEMPLATIZED FUNCTION IMPLEMENTATION///////////////

//creates a model based on a set of fields and a range of instances
template<typename It>
local_structure_model::local_structure_model(const field_set& fs,
                                             It from, It to,
                                             RandGen& _rng) :
    vector<dtree>(fs.raw_size()), _instance_length(fs.packed_width()),
    _ordering(make_counting_iterator(0), make_counting_iterator(int(size()))),
    _initial_deps(size()), _fields(fs), rng(_rng)
{
    super::iterator dtr = begin();

    if (!_fields.contin().empty() || !_fields.onto().empty()) {
        iptr_seq iptrs(make_transform_iterator(from, addressof<const instance>),
                       make_transform_iterator(to, addressof<const instance>));
        
        foreach(const field_set::onto_spec& o, _fields.onto()) { //onto vars
            int idx_base = distance(begin(), dtr);
            make_dtree(dtr++, o.tr->begin().number_of_children() + 1);

            for (field_set::arity_t i = 1;i < o.depth;++i, ++dtr) {
                make_dtree(dtr, 0);
                _initial_deps.insert(idx_base + i - 1, idx_base + i);
                //need to recursively split on gggparent, ... , gparent, parent
                rec_split_onto(iptrs.begin(), iptrs.end(),
                               idx_base, idx_base + i,
                               dtr->begin(), o.tr->begin());

                //make_dtree(dtr,o.branching);
            }
        }

        foreach(const field_set::contin_spec& c, _fields.contin()) { //contin vars
            int idx_base = distance(begin(), dtr);
            make_dtree(dtr++, 3); //contin arity is 3
            for (field_set::arity_t i = 1;i < c.depth;++i, ++dtr) {
                make_dtree(dtr, 3);
                _initial_deps.insert(idx_base + i - 1, idx_base + i); //add dep to parent

                //recursively split on gggparent, ... , gparent, parent
                /*rec_split_contin(iptrs.begin(),iptrs.end(),
                  idx_base,idx_base+i,dtr->begin());*/

                //alternatively, only split on parent
                rec_split_contin(iptrs.begin(), iptrs.end(),
                                 idx_base + i - 1, idx_base + i, dtr->begin());
            }
        }
    }

    //model disc & bool vars
    foreach(const field_set::disc_spec& d, _fields.disc_and_bits())
    make_dtree(dtr++, d.arity);

    //now that we have created all of the dtrees, construct a feasible order
    //that respects the intitial dependencies
    randomized_topological_sort(_initial_deps, _ordering.begin());
}

//instance_set is not const so that we can reorder it - the instances
//themselves shouldn't change
template<typename It>
void local_structure_probs_learning::operator()(const field_set& fs,
                                                It from, It to,
                                                local_structure_model& dst) const
{
    for_each(dst.begin(), dst.end(), make_counting_iterator(0),
             bind(&local_structure_probs_learning::rec_learn<It>, this, ref(fs),
                  from, to, _2, bind(&dtree::begin, _1)));
}

template<typename It>
void local_structure_probs_learning::rec_learn(const field_set& fs,
                                               It from, It to,
                                               int idx, dtree::iterator dtr) const
{
    if (dtr.is_childless()) { //a leaf
        while (from != to) {
            assert(fs.get_raw(*from, idx) < int(dtr->size()) - 1);
            ++(*dtr)[fs.get_raw(*from++,idx)];
        }

        //add +1 noisy
        /*if (dtr->size()>2) {
        for (dtree_node::iterator it=dtr->begin();it!=dtr->end();++it)
        ++(*it);
        }*/
        //std::cout << "XX " << (*dtr)[0] << " " << (*dtr)[1] << std::endl;
        //(*dtr)[0]=(*dtr)[0]/(*dtr)[0]+(*dtr)[1]

        dtr->back() = accumulate(dtr->begin(), --(dtr->end()), 0);
    } else { //an internal node (split) - sort [from,to) on the src idx
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

inline std::ostream& operator<<(std::ostream& o, const std::vector<int>& v)
{
    o << "< ";
    for (std::vector<int>::const_iterator it = v.begin();it != v.end();++it)
        o << (*it) << " ";
    o << ">";
    return o;
}

#endif
