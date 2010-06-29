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

#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/combo/type_tree.h>

#include "using.h"
#include "knob_mapper.h"

void set_stepsize(double new_ss);
void set_expansion(double new_ex);
void set_depth(int new_depth);

namespace moses
{

/**
 * Do the representation-building, create a field_set
 */
struct representation : public knob_mapper, boost::noncopyable {
    typedef eda::instance instance;

    typedef std::set<combo::vertex> operator_set;
    typedef std::set<combo::combo_tree, opencog::size_tree_order<combo::vertex> >
    combo_tree_ns_set;


    // Optional arguments are used only for actions/Petbrain
    representation(const reduct::rule& simplify,
                   const combo_tree& exemplar_,
                   const combo::type_tree& t,
                   opencog::RandGen& rng,
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
     * @return returns a copy of _exemplar, cleaned and reduced
     */
    combo_tree get_clean_exemplar(bool reduce);

    const field_set& fields() const {
        return _fields;
    }

    const combo_tree& exemplar() const {
        return _exemplar;
    }

protected:
    combo_tree _exemplar;
    field_set _fields;
    opencog::RandGen& rng;
    const reduct::rule* _simplify;
};

} //~namespace moses

#endif
