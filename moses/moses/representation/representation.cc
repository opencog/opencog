/*
 * opencog/learning/moses/representation/representation.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012 Poulin Holdings LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
 *            Predrag Janicic
 *            Linas Vepstas <linasvepstas@gmail.com>
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

#include <map>
#include <boost/thread.hpp>

#include <opencog/util/lazy_random_selector.h>
#include <opencog/util/oc_omp.h>

#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/reduct/meta_rules.h>
#include <opencog/comboreduct/reduct/logical_rules.h>
#include <opencog/comboreduct/reduct/general_rules.h>

#include "representation.h"
#include "build_knobs.h"

// uncomment the following to fine log the candidates before and after
// reduction during optimization
// #define __FINE_LOG_CND_REDUCED__

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

combo_tree type_to_exemplar(type_node type)
{
    switch(type) {
    case id::boolean_type: return combo_tree(id::logical_and);
    case id::contin_type: return combo_tree(id::plus);
    case id::enum_type: {
        combo_tree tr(id::cond);
        tr.append_child(tr.begin(), enum_t::get_random_enum());
        return tr;
    }
    case id::ill_formed_type:
        OC_ASSERT(false, "Error: the data type is incorrect, "
                  "perhaps it has not been possible to infer it from the "
                  "input table.");
    default: {
        std::stringstream ss;
        ss << "Error: type \"" << type << "\" not supported";
        OC_ASSERT(false, ss.str());
    }
    }
    return combo_tree();
}

representation::representation(const reduct::rule& simplify_candidate,
                               const reduct::rule& simplify_knob_building,
                               const combo_tree& exemplar_,
                               const type_tree& tt,
                               const operator_set& ignore_ops,
                               const combo_tree_ns_set* perceptions,
                               const combo_tree_ns_set* actions,
                               bool linear_contin,
                               float perm_ratio)
    : _exemplar(exemplar_),
      _simplify_candidate(&simplify_candidate),
      _simplify_knob_building(&simplify_knob_building)
{
    // Log before and after ... knob building can take a HUGE amount
    // of time for some situations ... capture these in the log file.
    logger().info() << "Start knob building, rep size="
                    << _exemplar.size()
                    << " complexity="
                    << tree_complexity(_exemplar);
    if (logger().isDebugEnabled()) {
        logger().debug() << "Building representation from exemplar: "
                         << _exemplar;
    }

    // Build the knobs.
    build_knobs(_exemplar, tt, *this, ignore_ops,
                perceptions, actions, linear_contin,
                stepsize, expansion, depth, perm_ratio);

    logger().info() << "After knob building, rep size="
                    << _exemplar.size()
                    << " complexity="
                    << tree_complexity(_exemplar);
    if (logger().isFineEnabled()) {
        logger().fine() << "Rep, after knob building: " << _exemplar;
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
    for (contin_v& v : contin)
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
    for (const disc_v& v : disc)
        tmp.insert(v.first);
    for (const contin_v& v : contin)
        tmp.insert(v.first);
    _fields = field_set(tmp.begin(), tmp.end());

    logger().info() << "Total number of field specs: " << tmp.size();

    // Build a reversed-lookup table for disc and contin knobs.
    // That is, given an iterator pointing into the exemplar, fetch the
    // correspinding knob (if any).
    size_t offset = 0;
    for (disc_map_cit dit = disc.cbegin(); dit != disc.cend(); dit++) {
        const disc_v& v = *dit;
        pre_it pit = v.second->get_loc();
        it_disc_knob[pit] = dit;
        // offset used to be calculated like this:
        // size_t offset = distance(disc.cbegin(), dit);
        // but distance() is 3000x slower!! The trick here is that the
        // knobs and fields are in exactly the same order, so this works.
        it_disc_idx[pit] = _fields.begin_disc_raw_idx() + offset;
        offset ++;
    }
    logger().info() << "Number of disc knobs mapped: " << disc.size();

    offset = 0;
    for (contin_map_cit cit = contin.cbegin(); cit != contin.cend(); cit++) {
        const contin_v& v = *cit;
        pre_it pit = v.second.get_loc();
        it_contin_knob[pit] = cit;
        // size_t offset = distance(contin.cbegin(), cit); per comments above.
        it_contin_idx[pit] = _fields.begin_contin_raw_idx() + offset;
        offset++;
    }
    logger().info() << "Number of contin knobs mapped: " << contin.size();
    logger().info() << "Field set, in bytes: " << _fields.byte_size();
    size_t is = sizeof(instance) + sizeof(packed_t) * _fields.packed_width();
    logger().info() << "One instance, in bytes: " << is;

    if (logger().isDebugEnabled()) {
        std::stringstream ss;
        ostream_prototype(ss << "Created prototype: ");
        logger().debug(ss.str());
    }

#ifdef EXEMPLAR_INST_IS_UNDEAD
    set_exemplar_inst();

    {
        std::stringstream ss;
        ss << "Exemplar instance: " << _fields.to_string(_exemplar_inst);
        logger().debug(ss.str());
    }
#endif // EXEMPLAR_INST_IS_UNDEAD
}

/// Turn the knobs on the representation, so that the knob settings match
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

    // cout << _fields.to_string(inst) << endl;
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
#ifdef __FINE_LOG_CND_REDUCED__
        // Save some cpu time by not even running this if-test.
        if (logger().isFineEnabled()) {
            logger().fine() << "Reduce "
                            << (knob_building? "(knob_building)" : "")
                            << " candidate: " << tr;
        }
#endif
        if (knob_building)
            (*get_simplify_knob_building())(tr);
        else
            (*get_simplify_candidate())(tr);
#ifdef __FINE_LOG_CND_REDUCED 
        if (logger().isFineEnabled()) {
            logger().fine() << "Reduced candidate: " << tr;
        }
#endif
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

/// Create a combo tree that corresponds to the instance inst.
///
/// This function is thread-safe; it does not modify the rep at all.
//
// Same function as get_candidate_lock but doesn't use lock and does not
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

/// Copy (append) *src, with knobs turned according 'inst', to be a
/// child of parent_dst.  If *src is not null_vertex, then repeat
/// recursively, using the appended child as new parent_dst.  If
/// 'candidate' is empty (i.e. parent_dst is invalid) then copy *src
/// (turned according to inst) as root of candidate
void representation::get_candidate_rec(const instance& inst,
                                       combo_tree::iterator src,
                                       combo_tree::iterator parent_dst,
                                       combo_tree& candidate) const
{
    typedef combo_tree::iterator pre_it;
    typedef combo_tree::sibling_iterator sib_it;

    // recursive call on the children of src to parent_dst
    auto recursive_call = [&inst, &candidate, this](pre_it new_parent_dst,
                                                    pre_it src)
    {
        for (sib_it src_child = src.begin(); src_child != src.end(); ++src_child)
            get_candidate_rec(inst, src_child, new_parent_dst, candidate);
    };

    // Find the knob associated to src (if any)
    disc_map_cit dcit = find_disc_knob(src);
    if (dcit != disc.end()) {
        int d = _fields.get_raw(inst, it_disc_idx.find(src)->second);
        pre_it new_src = dcit->second->append_to(candidate, parent_dst, d);
        if (_exemplar.is_valid(new_src))
            recursive_call(parent_dst, new_src);
        return;
    }

    contin_map_cit ccit = find_contin_knob(src);
    if (ccit != contin.end()) {
         contin_t c = _fields.get_contin(inst, it_contin_idx.find(src)->second);
         ccit->second.append_to(candidate, parent_dst, c);
         return;
    }

    // append v to parent_dst's children. If candidate is empty then
    // set it as head. Return the iterator pointing to the new content.
    auto append_child = [&candidate](pre_it parent_dst, const vertex& v)
    {
        return candidate.empty()? candidate.set_head(v)
            : candidate.append_child(parent_dst, v);
    };

    // There was no knob.  Just copy.
    recursive_call(append_child(parent_dst, *src), src);
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
    for (disc_v& v : disc)
        v.second->clear_exemplar();
    for (contin_v& v : contin)
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
