/*
 * opencog/learning/moses/representation/representation.h
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

#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/type_checker/type_tree.h>

#include "knob_mapper.h"

namespace opencog { namespace moses {

void set_stepsize(double new_ss);
void set_expansion(double new_ex);
void set_depth(int new_depth);

/**
 * determine the initial exemplar given its output type
 */
combo_tree type_to_exemplar(type_node type);
        
/**
 * Do the representation-building, create a field_set
 */
struct representation : public knob_mapper, boost::noncopyable
{
    typedef std::set<combo::vertex> operator_set;
    typedef std::set<combo::combo_tree, size_tree_order<combo::vertex> >
    combo_tree_ns_set;

    // Optional arguments are used only for actions/Petbrain
    representation(const reduct::rule& simplify_candidate,
                   const reduct::rule& simplify_knob_building,
                   const combo_tree& exemplar_,
                   const combo::type_tree& t,
                   const operator_set& ignore_ops = operator_set(),
                   const combo_tree_ns_set* perceptions = NULL,
                   const combo_tree_ns_set* actions = NULL,
                   bool linear_contin = true,
                   float perm_ratio = 0.0);

    /**
     * Turn the knobs on this representation, so that they have the same
     * settings as those in the 'instance' argument.
     */
    void transform(const instance&);

    /**
     * Returns a clean and reduced version of the current exemplar.
     * This would typically be called after turning some of its knobs.
     *
     * @param reduce whether the combo_tree is reduced before it is returned
     * @param knob_building if true then _simplify_knob_building is used to
     *                      reduce, otherwise (the default) _simplify_candidate.
     * @return returns a copy of _exemplar, cleaned and reduced
     */
    combo_tree get_clean_exemplar(bool reduce, bool knob_building = false) const;

    /**
     * Helper of get_clean_exemplar and get_candidate.
     *
     * Return a clean and possibly reduced version of tr. If knob_building
     * is true then _simplify_knob_building is used to reduce, otherwise
     * (the default) _simplify_candidate.
     */
    void clean_combo_tree(combo_tree &tr, bool reduce,
                          bool knob_building = false) const;

    /**
     * Thread safe composition of transform and
     * get_clean_exemplar. Around for the time being just in case but
     * should be removed eventually.
     */
    combo_tree get_candidate_lock(const instance& inst, bool reduce);
    
    /**
     * Like get_candidate but without lock
     */
    combo_tree get_candidate(const instance& inst, bool reduce) const;

    // recursive helper
    void get_candidate_rec(const instance& inst,
                           combo_tree::iterator src,
                           combo_tree::iterator parent_dst,
                           combo_tree& candidate) const;
    
    //* return _simplify_candidate
    const reduct::rule* get_simplify_candidate() const {
        return _simplify_candidate;
    }
    //* return _simplify_knob_building
    const reduct::rule* get_simplify_knob_building() const {
        return _simplify_knob_building;
    }

    const field_set& fields() const {
        return _fields;
    }

    field_set& fields() {
        return _fields;
    }

    const combo_tree& exemplar() const {
        return _exemplar;
    }

#ifdef EXEMPLAR_INST_IS_UNDEAD
    const instance& exemplar_inst() const {
        return _exemplar_inst;
    }
    void clear_exemplar();
#endif

    /**
     * Output the prototype of the exemplar (works correctly only when
     * the exemplar is not yet changed).
     */
    template<typename Out>
    Out& ostream_prototype(Out& out, combo_tree::iterator it) const
    {
        typedef combo_tree::sibling_iterator sib_it;
        if (is_contin(*it)) { // contin
            contin_map_cit c_cit = find_contin_knob(it);
            out << (c_cit == contin.end() ? *it : c_cit->second.toStr());
        } else { // disc
            disc_map_cit d_cit = find_disc_knob(it);
            out << (d_cit == disc.end() ? *it : d_cit->second->toStr());
            if (d_cit != disc.end()) {
                // In the case of action_subtree_knob just return. (it
                // would be wrong to do a recursive call because it
                // might be on the subtree that has alreaby been
                // ostreamed). This is kinda hacky.
                if (d_cit->second.type() == typeid(action_subtree_knob)) {
                    return out;
                }
            }
        }

        // if null_vertex then print its child instead
        if (*it == id::null_vertex) {
            OC_ASSERT(it.has_one_child());
            it = it.begin();
        }

        // recursive call on children
        if (not it.is_childless()) {
            out << "(";
            for (sib_it sib = it.begin(); sib != it.end();) {
                ostream_prototype(out, sib);
                if (++sib != it.end()) out << " ";
            }
            out << ")";
        }
        return out;
    }

    // like above but on the _exemplar
    template<typename Out>
    Out& ostream_prototype(Out& out) const
    {
        return ostream_prototype(out, _exemplar.begin());
    }

protected:
    combo_tree _exemplar;     // contains the prototype of the
                              // exemplar used to generate the deme

#ifdef EXEMPLAR_INST_IS_UNDEAD
    void set_exemplar_inst();
    instance _exemplar_inst; // instance corresponding to the exemplar
                             // @todo: it is not sure whether we need
                             // that because it is assumed that the
                             // instance of the exemplar is null
#endif

    field_set _fields;
    const reduct::rule* _simplify_candidate; // used to simplify candidates
    const reduct::rule* _simplify_knob_building; // used to simplify
                                                 // during knob
                                                 // building
    mutable boost::mutex tranform_mutex;
};

// This helper seems to be needed to unconfuse the compiler.
inline std::ostream& operator<<(std::ostream& out,
                                const opencog::moses::representation& r)
{
    return r.ostream_prototype(out);
}

} //~namespace moses
} //~namespace opencog

#endif
