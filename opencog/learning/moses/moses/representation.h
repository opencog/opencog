/*
 * opencog/learning/moses/moses/representation.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
 *            Predrag Janicic
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
#ifndef _MOSES_REPRESENTATION_H
#define _MOSES_REPRESENTATION_H

#include <boost/utility.hpp>

#include <boost/thread.hpp>

#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/combo/type_tree.h>

#include "using.h"
#include "knob_mapper.h"

namespace opencog { namespace moses {

void set_stepsize(double new_ss);
void set_expansion(double new_ex);
void set_depth(int new_depth);

/**
 * Do the representation-building, create a field_set
 */
struct representation : public knob_mapper, boost::noncopyable {
    typedef eda::instance instance;

    typedef std::set<combo::vertex> operator_set;
    typedef std::set<combo::combo_tree, size_tree_order<combo::vertex> >
    combo_tree_ns_set;


    // Optional arguments are used only for actions/Petbrain
    representation(const reduct::rule& simplify_candidate,
                   const reduct::rule& simplify_knob_building,
                   const combo_tree& exemplar_,
                   const combo::type_tree& t,
                   RandGen& rng,
                   const operator_set& ignore_ops = operator_set(),
                   const combo_tree_ns_set* perceptions = NULL,
                   const combo_tree_ns_set* actions = NULL);


    void transform(const instance&);
    void clear_exemplar();

    /**
     * returns a clean and reduced version of the current exemplar
     * (usually after turning some of its knobs)
     *
     * @param reduce whether the combo_tree is reduced before it is returned
     * @param knob_building if true then _simplify_knob_building is used to
     *                      reduce, otherwise (the default) _simplify_candidate.
     * @return returns a copy of _exemplar, cleaned and reduced
     */
    combo_tree get_clean_exemplar(bool reduce, bool knob_building = false);

    /**
     * Thread safe composition of transform and get_clean_exemplar
     */
    combo_tree get_candidate(const instance& inst, bool reduce);

    // return _simplify_candidate
    const reduct::rule* get_simplify_candidate() const {
        return _simplify_candidate;
    }
    // return _simplify_knob_building
    const reduct::rule* get_simplify_knob_building() const {
        return _simplify_knob_building;
    }

    const field_set& fields() const {
        return _fields;
    }

    const combo_tree& exemplar() const {
        return _exemplar;
    }
    
    const instance& exemplar_inst() const {
        return _exemplar_inst;
    }

    /**
     * Output the prototype of the exemplar (works correctly only when
     * the exemplar is yet unchanged).
     */
    template<typename Out>
    Out& ostream_prototype(Out& out, combo_tree::iterator it) const {
        typedef combo_tree::sibling_iterator sib_it;
        if(is_contin(*it)) { // contin
                contin_map_cit c_cit = find_contin_knob(it);
                out << (c_cit == contin.end() ? *it : c_cit->second.toStr());
        } else { // disc
            disc_map_cit d_cit = find_disc_knob(it);
            out << (d_cit == disc.end() ? *it : d_cit->second->toStr());
        }
        // if null_vertex then print its child instead
        if(*it == id::null_vertex) {
            OC_ASSERT(it.has_one_child());
            it = it.begin();
        }
        // recursive call on children            
        if(!it.is_childless()) {
            out << "(";
            for(sib_it sib = it.begin(); sib != it.end();) {
                ostream_prototype(out, sib);            
                if(++sib != it.end()) out << " ";
            }
            out << ")";
        }
        return out;
    }

    // like above but on the _exemplar
    template<typename Out>
    Out& ostream_prototype(Out& out) const {
        return ostream_prototype(out, _exemplar.begin());
    }

protected:
    void set_exemplar_inst();

    /**
     * Helper of get_clean_exemplar and get_candidate.
     *
     * Return a clean and possibly reduced version of tr. If knob_building
     * is true then _simplify_knob_building is used to reduce, otherwise
     * (the default) _simplify_candidate.
     */
    combo_tree get_clean_combo_tree(combo_tree tr, bool reduce,
                                    bool knob_building = false) const;

    combo_tree _exemplar;     // contains the prototype of the
                              // exemplar used to generate the deme

    instance _exemplar_inst; //instance corresponding to the exemplar
                             //@todo: it is not sure whether we need
                             //that because it is assumed that the
                             //instance of the exemplar is null
    field_set _fields;
    RandGen& rng;
    const reduct::rule* _simplify_candidate; // used to simplify candidates
    const reduct::rule* _simplify_knob_building; // used to simplify
                                                 // during knob
                                                 // building
    mutable boost::mutex tranform_mutex;
};

} //~namespace moses
} //~namespace opencog

#endif
