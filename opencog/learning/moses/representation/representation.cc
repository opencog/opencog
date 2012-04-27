/*
 * opencog/learning/moses/representation/representation.cc
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
#include <boost/range/algorithm/find_if.hpp>

#include <opencog/util/lazy_random_selector.h>

#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/reduct/meta_rules.h>
#include <opencog/comboreduct/reduct/logical_rules.h>
#include <opencog/comboreduct/reduct/general_rules.h>

#include "../moses/using.h"
#include "representation.h"
#include "build_knobs.h"

namespace opencog { namespace moses {

// Stepsize should be roughly the standard-deviation of the expected
// distribution of the contin variables.
//
// XXX TODO: One might think that varying the stepsize, i.e. shrinking
// it, as the optimizers tune into a specific value, would be a good
// thing (so that the optimizer could tune to a more precise value).
// Unfortunately, a simple experiment in tuning (see below, surrounded
// by "#if 0") didn't work; it didn't find better answers, and it
// lengthened running time.
static contin_t stepsize = 1.0;

// Expansion factor should be 1 or greater; it should never be less
// than one. Optimal values are probably 1.5 to 2.0.
static contin_t expansion = 2.0;

// By default, contin knobs will have 5 "pseudo-bits" of binary
// precision. Roughly speaking, they can hold 2^5 = 32 different
// values, or just a little better than one decimal place of precision.
static int depth = 5;

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
                               const operator_set& ignore_ops,
                               const combo_tree_ns_set* perceptions,
                               const combo_tree_ns_set* actions)
    : _exemplar(exemplar_),
      _simplify_candidate(&simplify_candidate),
      _simplify_knob_building(&simplify_knob_building)
{
    {
        std::stringstream ss;
        ss << "Building representation from exemplar: " << _exemplar;
        logger().debug(ss.str());
    }

    // Build the knobs.
    build_knobs(_exemplar, tt, *this, ignore_ops,
                perceptions, actions,
                stepsize, expansion, depth);

    {
        std::stringstream ss;
        ss << "Created prototype: ";
        ostream_prototype(ss);
        logger().debug(ss.str());
    }

#if 0
    // Attempt to adjust the contin spec step size to a value that is
    // "most likely to be useful" for exploring the neighborhood of an
    // exemplar. Basically, we try to guess what step size we were last
    // at when we modified this contin; we do this by lopping off
    // factors of two until we've matched the previous pecision.
    // This way, when exploring in a deme, we explore values near the
    // old value, with appropriate step sizes.
    //
    // Unfortunately, experiments seem to discredit this idea: it often
    // slows down the search, and rarely/never seems to provide better
    // answers.  Leaving this commented out for now.
    contin_map new_cmap;
    foreach (contin_v& v, contin)
    {
        field_set::contin_spec cspec = v.first;
        contin_t remain = fabs(cspec.mean);
        remain -= floor(remain + 0.01f);
        contin_t new_step = remain;
        if (remain < 0.01f) new_step = stepsize;
        else remain -= new_step;
        while (0.01f < remain)
        {
            new_step *= 0.5f;
            if (new_step < remain + 0.01f) remain -= new_step;
        }
        cspec.step_size = new_step;
        new_cmap.insert(make_pair(cspec, v.second));
    }
    contin = new_cmap;
#endif

    // Convert the knobs into a field set.
    std::multiset<field_set::spec> tmp;
    foreach (const disc_v& v, disc)
        tmp.insert(v.first);
    foreach (const contin_v& v, contin)
        tmp.insert(v.first);
    _fields = field_set(tmp.begin(), tmp.end());

    // build mapping from combo tree iterator to knobs and idx
    for (pre_it it = _exemplar.begin(); it != _exemplar.end(); ++it) {
        // find disc knobs
        disc_map_cit d_cit = boost::find_if(disc, [&it](const disc_v& v) {
                return v.second->get_loc() == it; });
        if (d_cit != disc.end()) {
            it_disc_knob[it] = d_cit;
            it_disc_idx[it] = _fields.begin_disc_raw_idx()
                + distance(disc.cbegin(), d_cit);
        } else {
            // find contin knobs
            contin_map_cit c_cit =
                boost::find_if(contin, [&it](const contin_v& v) {
                        return v.second.get_loc() == it; });
            if (c_cit != contin.end()) {
                it_contin_knob[it] = c_cit;
                it_contin_idx[it] = distance(contin.cbegin(), c_cit);
            }
        }
    }

    
#ifdef EXEMPLAR_INST_IS_UNDEAD
    set_exemplar_inst();

    {
        std::stringstream ss;
        ss << "Exemplar instance: " << _fields.stream(_exemplar_inst);
        logger().debug(ss.str());
    }
#endif // EXEMPLAR_INST_IS_UNDEAD
}

/// Turn the knobs on the representation, so thaat the knob settings match
/// the instance supplied as the argument.
void representation::transform(const instance& inst)
{
    // XXX TODO need to add support for "term algebra" knobs

    contin_map_it ckb = contin.begin();
    for (field_set::const_contin_iterator ci = _fields.begin_contin(inst);
            ci != _fields.end_contin(inst); ++ci, ++ckb) {
        ckb->second.turn(*ci);
        //_exemplar.validate();
    }

    // cout << _fields.stream(inst) << endl;
    disc_map_it dkb = disc.begin();
    for (field_set::const_disc_iterator di = _fields.begin_disc(inst);
            di != _fields.end_disc(inst); ++di, ++dkb) {
        dkb->second->turn(*di);
        //_exemplar.validate();
    }

    for (field_set::const_bit_iterator bi = _fields.begin_bit(inst);
            bi != _fields.end_bit(inst); ++bi, ++dkb) {
        dkb->second->turn(*bi);
    }
    //cout << _exemplar << endl;
    // std::cout << "New exemplar (after build): " << _exemplar << std::endl;
}

combo_tree representation::get_clean_exemplar(bool reduce,
                                              bool knob_building) const
{
    // Make a copy -- expensive! but necessary.
    combo_tree tr = exemplar();
    clean_combo_tree(tr, reduce, knob_building);
    return tr;
}

void representation::clean_combo_tree(combo_tree &tr,
                                      bool reduce,
                                      bool knob_building) const
{
    using namespace reduct;

    // Remove null vertices.
    clean_reduce(tr);

    if (reduce) { //reduce
        if (logger().isFineEnabled()) {
            logger().fine() << "Reduce (knob=" << knob_building
                            << ") candidate: " << tr;
        }
        if (knob_building)
            (*get_simplify_knob_building())(tr);
        else
            (*get_simplify_candidate())(tr);
    }
}

/// Create a combo tree that corresponds to the instance inst.
///
/// This is done by twiddling the knobs on the representation so that
/// it matches the supplied inst, copying the rep to an equivalent,
/// but knob-less combo tree, and returning that.  Optionally, if the
/// 'reduce' flag is set, then the tree is reduced.
///
/// Warning: use get_candidate instead which is both more efficient
/// and scales better on multi-proc (as it is locker free)
combo_tree representation::get_candidate_lock(const instance& inst, bool reduce)
{
    // In order to make this method thread-safe, a copy of the exemplar
    // must be made under the lock, after the transform step. 
    // Unfortunately, copying the exemplar is expensive, but seemingly
    // unavoidable: the knob mapper only works for the member _exemplar.
    // Thus we cannot have "const combo_tree &tr = exemplar();"
    boost::mutex::scoped_lock lock(tranform_mutex);
    transform(inst);
    combo_tree tr = exemplar();  // make copy before unlocking.
    lock.unlock();
    clean_combo_tree(tr, reduce);
    return tr;
}

// Same function as get_candidate but doesn't use lock and does not
// modify _exemplar, instead it build the combo tree from scratch
combo_tree representation::get_candidate(const instance& inst, bool reduce) const
{
    combo_tree candidate;
    get_candidate_rec(inst, _exemplar.begin(), candidate.end(), candidate);
    // this can probably be simplified, it doesn't need to remove
    // null_vertices anymore
    clean_combo_tree(candidate, reduce);
    return candidate;
}

// Append *src (turned according inst) as child of parent_dst and, in
// case it's not null_vertex, repeat recursively using that appended
// child as new parent_dst. If candidate is empty (parent_dst is
// invalid) then copy *src (turned according to inst) as root of
// candidate
void representation::get_candidate_rec(const instance& inst,
                                       combo_tree::iterator src,
                                       combo_tree::iterator parent_dst,
                                       combo_tree& candidate) const
{
    typedef combo_tree::iterator pre_it;
    typedef combo_tree::sibling_iterator sib_it;

    // recursive call on the children of src to parent_dst
    auto recursive_call = [&inst, &candidate, this](pre_it new_parent_dst,
                                                    pre_it src) {
        for(sib_it src_child = src.begin(); src_child != src.end(); ++src_child)
            get_candidate_rec(inst, src_child, new_parent_dst, candidate);        
    };

    // append v to parent_dst's children. If candidate is empty then
    // set it as head. Return the iterator pointing to the new content.
    auto append_child = [&candidate](pre_it parent_dst, const vertex& v) {
        return candidate.empty()? candidate.set_head(v)
            : candidate.append_child(parent_dst, v);
    };

    // find the knob associated to src (if any)
    disc_map_cit dcit = find_disc_knob(src);
    if (dcit == disc.end()) {
        contin_map_cit ccit = find_contin_knob(src);
        if (ccit == contin.end()) // no knob found      
            recursive_call(append_child(parent_dst, *src), src);
        else { // contin knob found
            contin_t c = _fields.get_contin(inst, it_contin_idx.find(src)->second);
            ccit->second.append_to(candidate, parent_dst, c);
        }
    } else { // disc knob found
        int d = _fields.get_raw(inst, it_disc_idx.find(src)->second);
        pre_it new_src = dcit->second->append_to(candidate, parent_dst, d);
        if (_exemplar.is_valid(new_src))
            recursive_call(parent_dst, new_src);
    }
}

#ifdef EXEMPLAR_INST_IS_UNDEAD
// XXX This is dead code, no one uses it, and looking at the below, it
// looks inconsistent to me. I'm going to leave it here for a while, but
// it should be removed by 2013 or 2014 if not sooner...

// XXX why are we clearing this, instead of setting it back to the 
// _exemplar_inst ??? XXX is this broken??
//
// XXX Note that the clear_exemplar() methods on the knobs are probably
//  not needed either!?
void representation::clear_exemplar()
{
    foreach(disc_v& v, disc)
        v.second->clear_exemplar();
    foreach(contin_v& v, contin)
        v.second.clear_exemplar();
}

// What is this doing ? seems to be clearing things out, why do we need this?
// XXX that, and contin seems to be handled inconsistently with disc...
// I mean, shouldn't we be setting the exemplar_inst fields so that
// they match the exmplar?  Do we even need the exemplar_inst for anything?
void representation::set_exemplar_inst()
{
    OC_ASSERT(_exemplar_inst.empty(),
              "Should be called only once,"
              " therefore _exemplar_inst should be empty");
    _exemplar_inst.resize(_fields.packed_width());

    // @todo: term algebras
    // bit
    for(field_set::bit_iterator it = _fields.begin_bit(_exemplar_inst);
        it != _fields.end_bit(_exemplar_inst); ++it)
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
#endif // EXEMPLAR_INST_IS_UNDEAD

} // ~namespace moses
} // ~namespace opencog
