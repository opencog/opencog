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

#include "field_set.h"
#include "knobs.h"

namespace opencog { namespace moses {

/**
 * maps disc_spec (resp. contin_spec) to actual disc_knob
 * (resp. contin_knob). The disc mapping includes bits.
 */
struct knob_mapper
{
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

    // find the disc knob corresponding to 'it'
    disc_map_cit find_disc_knob(combo_tree::iterator it) const {
        for(disc_map_cit d_cit = disc.begin(); d_cit != disc.end(); ++d_cit) {
            if(d_cit->second->get_loc() == it)
                return d_cit;
        }
        return disc.end();
    }
    // find the contin knob corresponding to 'it'
    contin_map_cit find_contin_knob(combo_tree::iterator it) const {
        for(contin_map_cit c_cit = contin.begin(); c_cit != contin.end();
            ++c_cit) {
            if(c_cit->second.get_loc() == it)
                return c_cit;
        }
        return contin.end();
    }
};

} //~namespace moses
} //~namespace opencog

#endif
