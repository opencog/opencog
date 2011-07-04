/*
 * opencog/learning/moses/eda/field_set.cc
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
#include "field_set.h"
#include <sstream>
#include <opencog/util/dorepeat.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/oc_assert.h>
#include <opencog/util/iostreamContainer.h>

namespace opencog { 
namespace eda {

const disc_t field_set::contin_spec::Stop = 0;
const disc_t field_set::contin_spec::Left = 1;
const disc_t field_set::contin_spec::Right = 2;

const disc_t field_set::onto_spec::Stop = 0;

field_set& field_set::operator=(const field_set& rhs)
{
    _fields = rhs._fields;
    _disc = rhs._disc;
    _contin = rhs._contin;
    _onto = rhs._onto;
    _nbool = rhs._nbool;
    _contin_start = _fields.begin() + distance(rhs._fields.begin(),
                                               rhs._contin_start);
    _disc_start = _fields.begin() + distance(rhs._fields.begin(),
                                             rhs._disc_start);
    return *this;
}
bool field_set::operator==(const field_set& rhs) const
{
    return (_disc == rhs._disc && _contin == rhs._contin &&
            _onto == rhs._onto && _nbool == rhs._nbool);
}

const onto_t& field_set::get_onto(const instance& inst, size_t idx) const
{
    size_t raw_idx = onto_to_raw_idx(idx);
    //walk down the tree to get the appropriate onto_t
    const onto_spec& o = _onto[idx];
    onto_tree::iterator it = o.tr->begin();
    for (arity_t i = 0;i < o.depth;++i) {
        disc_t raw_value = get_raw(inst, raw_idx + i);
        if (raw_value == onto_spec::Stop) {
            break;
        } else {
            it = o.tr->child(it, onto_spec::to_child_idx(raw_value));
        }
    }
    return *it;
}

contin_t field_set::get_contin(const instance& inst, size_t idx) const
{
    size_t raw_idx = contin_to_raw_idx(idx);
    //start with the mean a walk down to the res
    const contin_spec& c = _contin[idx];
    contin_stepper stepper(c);
    for (arity_t i = 0;i < c.depth;++i) {
        disc_t direction = get_raw(inst, raw_idx + i);
        if (direction == contin_spec::Stop) {
            break;
        } else if (direction == contin_spec::Left) {
            stepper.left();
        } else { //direction==contin_spec::Right
            OC_ASSERT(direction == contin_spec::Right,
                             "direction should be set to contin_spec::Right (get_contin).");
            stepper.right();
        }
    }
    return stepper.value;
}

void field_set::set_contin(instance& inst, size_t idx, contin_t target) const
{
    size_t raw_idx = contin_to_raw_idx(idx);
    const contin_spec& c = _contin[idx];

    //use binary search to assign to the nearest value
    contin_t best_distance = fabs(c.mean - target);
    arity_t best_depth = 0;
    contin_stepper stepper(c);
    for (arity_t i = 0;i<c.depth && best_distance>c.epsilon();++i) {
        //take a step in the correct direction
        if (target < stepper.value) { //go left
            stepper.left();
            set_raw(inst, raw_idx + i, contin_spec::Left);
        } else { //target>stepper.value - go right
            stepper.right();
            set_raw(inst, raw_idx + i, contin_spec::Right);
        }
        if (fabs(stepper.value - target) < best_distance) {
            best_distance = fabs(stepper.value - target);
            best_depth = i + 1;
        }
    }

    //backtrack up to the best depth
    for (arity_t i = best_depth;i < c.depth;++i)
        set_raw(inst, raw_idx + i, contin_spec::Stop);
}

//note to self (Moshe): changed this on 4/23 - its confusing that
//stream and stream_raw give different orderings among the various
//types of vars - what was I thinking?
std::string field_set::stream(const instance& inst) const
{
    std::stringstream ss;
    ss << "[";
    ostreamContainer(ss, begin_onto(inst), end_onto(inst), "#", "#", "", false);
    ostreamContainer(ss, begin_contin(inst), end_contin(inst), "|", "|", "", false);
    ostreamContainer(ss, begin_disc(inst), end_disc(inst), " ", " ", "", false);
    ostreamContainer(ss, begin_bits(inst), end_bits(inst), "", "", "", false);
    ss << "]";
    return ss.str();
}

std::string field_set::stream_raw(const instance& inst) const
{
    std::stringstream ss;
    ostreamContainer(ss, begin_raw(inst), end_raw(inst), "", "[", "]");
    return ss.str();
}

void field_set::build_spec(const spec& s, size_t n)
{
    if (const onto_spec* os = boost::get<onto_spec>(&s)) {
        build_onto_spec(*os, n);
    } else if (const contin_spec* cs = boost::get<contin_spec>(&s)) {
        build_contin_spec(*cs, n);
    } else if (const disc_spec* ds = boost::get<disc_spec>(&s)) {
        build_disc_spec(*ds, n);
    } else {
        OC_ASSERT(false, "This spec is NULL or unknown");
    }
}

void field_set::build_disc_spec(const disc_spec& ds, size_t n)
{
    arity_t width = nbits_to_pack(ds.arity);
    size_t base = back_offset();
    for (size_t idx = 0;idx < n;++idx)
        _fields.push_back(field(width,
                                (base + idx*width) / bits_per_packed_t,
                                (base + idx*width) % bits_per_packed_t));
    _disc.insert(_disc.end(), n, ds);
    if (width == 1)
        _nbool += n;
}

void field_set::build_contin_spec(const contin_spec& cs, size_t n)
{
    //depth must be a power of 2
    OC_ASSERT(cs.depth == next_power_of_two(cs.depth),
              "depth must be a power of 2 and it is %d", 
              cs.depth); 
    //all have arity of 3 (left, right, or stop) and hence are 2 wide
    size_t base = back_offset(), width = 2;
    dorepeat(n*cs.depth) {
        _fields.push_back(field(width, base / bits_per_packed_t,
                                base % bits_per_packed_t));
        base += width;
    }
    _contin.insert(_contin.end(), n, cs);
}

void field_set::build_onto_spec(const onto_spec& os, size_t n)
{
    size_t base = back_offset(), width = nbits_to_pack(os.branching);
    size_t total_width = size_t((width * os.depth - 1) /
                                bits_per_packed_t + 1) * bits_per_packed_t;
        
    dorepeat(n) {
        dorepeat(os.depth) {
            _fields.push_back(field(width, base / bits_per_packed_t,
                                    base % bits_per_packed_t));
            base += width;
        }
        base += total_width - (os.depth * width); //onto vars must pack evenly
    }
    _onto.insert(_onto.end(), n, os);
}

} // ~namespace eda
} // ~namespace opencog
