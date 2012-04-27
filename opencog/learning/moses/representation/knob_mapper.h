/*
 * opencog/learning/moses/moses/knob_mapper.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2008-2012 OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Moshe Looks
 *            Nil Geisweiller
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
#ifndef _MOSES_KNOB_MAPPER_H
#define _MOSES_KNOB_MAPPER_H

#include <map>
#include <unordered_map>

#include <opencog/util/hashing.h>

#include "field_set.h"
#include "knobs.h"

namespace opencog { namespace moses {

/**
 * maps disc_spec (resp. contin_spec) to actual disc_knob
 * (resp. contin_knob). The disc mapping includes bits.
 *
 * In addition it maps combo_tree::iterators from the exemplar to
 * pairs of knob * raw_idx. The raw_idx will be used to access the
 * value stored in an instance.
 */
struct knob_mapper
{
    typedef combo_tree::iterator pre_it;
    
    // Important: knobs are kept sorted in an order consistant with
    // that of the field_set _fields that is constructed according to
    // their corresponding specs
    typedef std::multimap<field_set::disc_spec, disc_knob> disc_map;
    typedef disc_map::value_type disc_v;
    typedef disc_map::const_iterator disc_map_cit;
    typedef disc_map::iterator disc_map_it;

    typedef std::multimap<field_set::contin_spec, contin_knob> contin_map;
    typedef contin_map::value_type contin_v;
    typedef contin_map::const_iterator contin_map_cit;
    typedef contin_map::iterator contin_map_it;

    disc_map disc;              // constructed by build_knobs
    contin_map contin;          // constructed by build_knobs

    // map the address of the combo_tree node to its corresponding
    // knob. This can only work if the combo tree is immutable. Which
    // we assume in the new design (as we need that for thread safety)
    typedef std::unordered_map<pre_it, disc_map_cit,
                               obj_ptr_hash<pre_it>> it_disc_knob_map;
    typedef std::unordered_map<pre_it, contin_map_cit,
                               obj_ptr_hash<pre_it>> it_contin_knob_map;
    typedef std::unordered_map<pre_it, int,
                               obj_ptr_hash<pre_it>> it_disc_idx_map;
    typedef std::unordered_map<pre_it, int,
                               obj_ptr_hash<pre_it>> it_contin_idx_map;

    it_disc_knob_map it_disc_knob;
    it_contin_knob_map it_contin_knob;
    it_disc_idx_map it_disc_idx;
    it_contin_idx_map it_contin_idx;
    
    // find the disc knob corresponding to 'it'. Return disc.end() if
    // none found
    disc_map_cit find_disc_knob(const pre_it& it) const {
        it_disc_knob_map::const_iterator res = it_disc_knob.find(it);
        return res == it_disc_knob.end() ? disc.cend() : res->second;
    }
    // find the contin knob corresponding to 'it'. Return contin.end()
    // if none found
    contin_map_cit find_contin_knob(const pre_it& it) const {
        it_contin_knob_map::const_iterator res = it_contin_knob.find(it);
        return res == it_contin_knob.end() ? contin.cend() : res->second;
    }
};

} //~namespace moses
} //~namespace opencog

#endif
