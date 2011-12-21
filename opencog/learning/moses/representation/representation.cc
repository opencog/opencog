/*
 * opencog/learning/moses/moses/representation.cc
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

#include <boost/thread.hpp>

#include <opencog/util/lazy_random_selector.h>

#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/reduct/meta_rules.h>
#include <opencog/comboreduct/reduct/logical_rules.h>
#include <opencog/comboreduct/reduct/general_rules.h>

#include "../moses/using.h"
#include "representation.h"
#include "build_knobs.h"

namespace opencog { namespace moses {

static contin_t stepsize = 1.0;
static contin_t expansion = 1.0;
static int depth = 4;

void set_stepsize(double new_ss)
{
    stepsize = new_ss;
}

void set_expansion(double new_ex)
{
    expansion = new_ex;
}

void set_depth(int new_depth)
{
    depth = new_depth;
}

representation::representation(const reduct::rule& simplify_candidate,
                               const reduct::rule& simplify_knob_building,
                               const combo_tree& exemplar_,
                               const type_tree& tt,
                               RandGen& _rng,
                               const operator_set& ignore_ops,
                               const combo_tree_ns_set* perceptions,
                               const combo_tree_ns_set* actions)
    : _exemplar(exemplar_), rng(_rng),
      _simplify_candidate(&simplify_candidate),
      _simplify_knob_building(&simplify_knob_building)
{
    // Logger
    {
        std::stringstream ss;
        ss << "Representation building from exemplar: " << _exemplar;
        logger().debug(ss.str());
    }
    // ~Logger

    //build the knobs
    build_knobs(rng, _exemplar, tt, *this, ignore_ops,
                perceptions, actions,
                stepsize, expansion, depth);

    // Logger
    {
        std::stringstream ss;
        ss << "Created prototype: ";
        ostream_prototype(ss);
        logger().debug(ss.str());
    }
    // ~Logger
    
    //handle knob merging

    //convert the knobs into a field specification
    std::multiset<field_set::spec> tmp;
    foreach(const disc_v& v, disc)
        tmp.insert(v.first);
    foreach(const contin_v& v, contin)
        tmp.insert(v.first);
    _fields = field_set(tmp.begin(), tmp.end());

    set_exemplar_inst();

    // Logger
    {
        std::stringstream ss;
        ss << "Exemplar instance: " << _fields.stream(_exemplar_inst);
        logger().debug(ss.str());
    }
    // ~Logger    
}

void representation::transform(const instance& inst)
{
    contin_map_it ckb = contin.begin();
    for (field_set::const_contin_iterator ci = _fields.begin_contin(inst);
            ci != _fields.end_contin(inst);++ci, ++ckb) {
        ckb->second.turn(*ci);
        //_exemplar.validate();
    }

    //need to add first term & then contin
    //cout << _fields.stream(inst) << endl;
    disc_map_it dkb = disc.begin();
    for (field_set::const_disc_iterator di = _fields.begin_disc(inst);
            di != _fields.end_disc(inst);++di, ++dkb) {
        dkb->second->turn(*di);
        //_exemplar.validate();
    }
    for (field_set::const_bit_iterator bi = _fields.begin_bits(inst);
            bi != _fields.end_bits(inst);++bi, ++dkb) {
        dkb->second->turn(*bi);
    }
    //cout << _exemplar << endl;
    // std::cout << "New exemplar (after build): " << _exemplar << std::endl;

}

void representation::clear_exemplar()
{
    foreach(disc_v& v, disc)
        v.second->clear_exemplar();
    foreach(contin_v& v, contin)
        v.second.clear_exemplar();
}


combo_tree representation::get_clean_exemplar(bool reduce,
                                              bool knob_building) const
{
    return get_clean_combo_tree(exemplar(), reduce, knob_building);
}

combo_tree representation::get_clean_combo_tree(combo_tree tr,
                                                bool reduce,
                                                bool knob_building) const
{
    using namespace reduct;

    clean_reduce(tr); //remove null vertices

    if(reduce) { //reduce
        // Logger
        if(logger().getLevel() >= Logger::FINE) {
            std::stringstream ss;
            ss << "Reduce candidate: " 
               << tr;
            logger().fine(ss.str());
        }
        // ~Logger
        if(knob_building)
            (*get_simplify_knob_building())(tr); 
        else
            (*get_simplify_candidate())(tr); 
    }

    return tr;
}

combo_tree representation::get_candidate(const instance& inst, bool reduce)
{
    // thread safe transform. If that turns out to be a bottle neck,
    // one should copy the exemplar before tranformation and do the
    // transformation (unlocked) on that copy
    boost::mutex::scoped_lock lock(tranform_mutex);
    transform(inst);
    combo_tree tr = exemplar();
    lock.unlock();

    return get_clean_combo_tree(tr, reduce);
}

void representation::set_exemplar_inst()
{
    OC_ASSERT(_exemplar_inst.empty(),
              "Should be called only once,"
              " therefore _exemplar_inst should be empty");
    _exemplar_inst.resize(_fields.packed_width());

    // @todo: term algebras
    // bit
    for(field_set::bit_iterator it = _fields.begin_bits(_exemplar_inst);
        it != _fields.end_bits(_exemplar_inst); ++it)
        *it = false;
    // disc
    for(field_set::disc_iterator it = _fields.begin_disc(_exemplar_inst);
        it != _fields.end_disc(_exemplar_inst); ++it)
        *it = 0;
    // contin
    contin_map_cit c_cit = contin.begin();
    for(field_set::contin_iterator it = _fields.begin_contin(_exemplar_inst);
        it != _fields.end_contin(_exemplar_inst); ++it, ++c_cit) {
        // this should actually set the contin knob to Stop ...
        // because the mean of the associated contin field is
        // equal to the initial contin value
        *it = get_contin(*c_cit->second.get_loc());
    }
}

} // ~namespace moses
} // ~namespace opencog
