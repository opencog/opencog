/** neighborhood_sampling.h --- 
 *
 * Copyright (C) 2010 Moshe Looks, OpenCog Foundation, Xiaohui Liu
 *
 * Author: Moshe Looks, Nil Geisweiller, Xiaohui Liu
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
#ifndef _OPENCOG_NEIGHBORHOOD_SAMPLING_H
#define _OPENCOG_NEIGHBORHOOD_SAMPLING_H

#include <iostream>
#include <algorithm>
#include <limits>

#include <boost/math/special_functions/binomial.hpp>
#include <boost/numeric/conversion/cast.hpp>

#include <opencog/util/dorepeat.h>
#include <opencog/util/lazy_random_selector.h>

#include "using.h"
#include "../eda/initialization.h"

namespace moses
{

using opencog::pow2;
using boost::math::binomial_coefficient;
using boost::numeric_cast;
using boost::numeric::positive_overflow;
using std::numeric_limits;

/**
 * This procedure generate the initial deme randomly
 *
 * @param fs  the deme
 * @param n   the size of deme
 * @param out deme iterator (where to store the instance)
 * @param end deme iterator, containing the end iterator,
 *            necessary to check that 'out' does not go out
 *            of the deme if it does so then an assert is raised
 */
template<typename Out>
void generate_initial_sample(const eda::field_set& fs, int n, Out out, Out end,
                             opencog::RandGen& rng)
{
    dorepeat(n) {

        eda::instance inst(fs.packed_width());

        eda::randomize(fs, inst, rng);

        //bias towards the exemplar instance
        for (eda::field_set::bit_iterator it = fs.begin_bits(inst);
             it != fs.end_bits(inst);++it)
            if (rng.randbool())
                *it = false;
        for (eda::field_set::disc_iterator it = fs.begin_disc(inst);
             it != fs.end_disc(inst);++it)
            if (rng.randbool())
                *it = 0;

        //add it
        OC_ASSERT(out != end);  // to avoid invalid memory write
        *out++ = inst;
    }
}

/**
 * It generates the contin neighbor with the haming distance from 
 * the given instance. For examples, if the contin[it.idx()] is encoded
 * with depth = 4,like (L R S S), so the neighbors with distance = 1 of it
 * are (R R S S), (L L S S),(L R L S),(L S S S) and (L R R S). And we 
 * randomly chose one of them to return.
 *
 * @todo: in order to increase syntactic vs semantics correlation one
 * may want to not consider contin neighbors which encodes to contin
 * with too alrge difference from the given instance. So for example
 * in the example given above we would ignore (R R S S).
 *
 * @param fs   deme
 * @param inst the instance will be modified is contin encoded with distance
 *             equal to n 
 * @param it   the contin iterator of the instance
 * @param n    the haming distance contin encode will be modified
 * @param rng  the random generator
 */
inline void generate_contin_neighbor(const eda::field_set& fs,
                                     eda::instance& inst, 
                                     eda::field_set::contin_iterator it, 
                                     unsigned n, opencog::RandGen& rng)
{
    size_t begin = fs.contin_to_raw_idx(it.idx());
    size_t num = fs.count_n_before_stop(inst, it.idx());
    size_t depth = fs.contin()[it.idx()].depth;
    // a random_selector is used not to pick up twice the same idx.
    // The max idx coresponds either to the first Stop, or, in case
    // there is no Stop, the last disc (i.e. either Left or Right)
    opencog::lazy_random_selector select(std::min(num + 1, depth), rng);

    for(unsigned i = n; i > 0; i--) {
        size_t r = select();
        eda::field_set::disc_iterator itr = fs.begin_raw(inst);        
        itr += begin + r;
        // case inst at itr is Stop
        if(*itr == eda::field_set::contin_spec::Stop) {
            *itr = rng.randbool() ?
                eda::field_set::contin_spec::Left:
                eda::field_set::contin_spec::Right;
            num++;
            select.reset_range(std::min(num + 1, depth));
        } 
        // case inst at itr is Left or Right
        else {
            // whether r corresponds to the last Left or Right disc
            bool before_stop = r + 1 == num;
            // whether we allow to turn it to stop
            bool can_be_stop = i <= select.count_n_free();
            if(before_stop && can_be_stop && rng.randbool()) {
                *itr = eda::field_set::contin_spec::Stop;
                select.reset_range(--num);
            } else 
                *itr = eda::field_set::contin_spec::switchLR(*itr);
        }
    }
}

/**
 * This procedure samples sample_size instances at distance n from an
 * instance considered as center (for isntance the exemplar). 
 *
 * @todo: onto fields are ignored for now
 *
 * @param fs            deme
 * @param n             distance
 * @param sample_size   number of instances to be generated
 * @param out           deme where to store the instances
 * @param end           deme iterator, containing the end iterator,
 *                      necessary to check that 'out' does not go out
 *                      of the deme if it does so then an assert is raised
 * @param rng           the random generator
 * @param center_inst   the center instance
 */
template<typename Out>
void sample_from_neighborhood(const eda::field_set& fs, unsigned n,
                              unsigned sample_size, Out out, Out end,
                              opencog::RandGen& rng,
                              const eda::instance & center_inst )
{
    OC_ASSERT(center_inst.size() == fs.packed_width(),
              "Please make sure that the center_inst"
              " have the same size with the field_set");

    unsigned dim = fs.dim_size();

    OC_ASSERT(n <= dim,
              "the sampling distance %u"
              " cannot be greater than the field dimension %u", n, dim);

    dorepeat(sample_size) {

        eda::instance new_inst(center_inst);
        opencog::lazy_random_selector select(dim, rng);

        for (unsigned i = 1;i <= n;) {
            size_t r = select();
            eda::field_set::bit_iterator itb = fs.begin_bits(new_inst);
            eda::field_set::disc_iterator itd = fs.begin_disc(new_inst);
            eda::field_set::contin_iterator itc = fs.begin_contin(new_inst);
            // modify bit
            if (r < fs.n_bits()) {
                itb += r;
                *itb = !(*itb);
                i++;
            // modify disc
            } else if (r >= fs.n_bits() && (r < (fs.n_bits() + fs.n_disc()))) {
                itd += r - fs.n_bits();
                int temp = 1 + rng.randint(itd.arity() - 1);
                if ( *itd == temp)
                    *itd = 0;
                else
                    *itd = temp;
                i++;
            // modify contin
            } else if ( r >= (fs.n_bits() + fs.n_disc())) {
                //cout << "i = " << i << "  r = " << r << endl;
                itc += r - fs.n_bits() - fs.n_disc();
                // @todo: now the distance is 1, choose the distance
                // of contin possibly different than 1
                generate_contin_neighbor(fs, new_inst, itc, 1, rng);
                i++;
            }
        }
        OC_ASSERT(out != end); // to avoid invalid memory write
        *out++ = new_inst;
        // cout << "********** Added instance:" << fs.stream(new_inst) << endl;
    }
}


/**
 * This procedure samples sample_size instances at distance n from the exemplar
 * (i.e., with n non-zero elements in the sequence)
 *
 * @param fs            deme
 * @param n             distance
 * @param sample_size   number of instances to be generated
 * @param out           deme iterator (where to store the instances)
 * @param end           deme iterator, containing the end iterator,
 *                      necessary to check that 'out' does not go out
 *                      of the deme if it does so then an assert is raised
 */
template<typename Out>
void sample_from_neighborhood(const eda::field_set& fs, unsigned n,
                              unsigned sample_size, Out out, Out end,
                              opencog::RandGen& rng)
{
    eda::instance inst(fs.packed_width());

    // reset all fields (contin and onto fields are ignored)
    for (eda::field_set::bit_iterator it = fs.begin_bits(inst);
            it != fs.end_bits(inst); ++it)
        *it = false;

    for (eda::field_set::disc_iterator it = fs.begin_disc(inst);
            it != fs.end_disc(inst); ++it)
        *it = 0;

    sample_from_neighborhood(fs, n, sample_size, out, end, rng, inst);
}


/**
 * Generates instances at distance n from an instance considered as
 * center (often the exemplar but not necessarily).
 * It calls a recursive function vary_n_knobs which varies
 * instance fields one by one (in all possible ways)
 *
 * @param fs            deme
 * @param n             distance
 * @param out           deme (where to store the instances)
 * @param end           deme iterator, containing the end iterator,
 *                      necessary to check that 'out' does not go out
 *                      of the deme if it does so then an assert is raised
 * @param center_inst   the center instance 
 */
template<typename Out>
void generate_all_in_neighborhood(const eda::field_set& fs, unsigned n,
                                  Out out, Out end,
                                  const eda::instance& center_inst )
{
    OC_ASSERT(center_inst.size() == fs.packed_width(),
              "the size of center_instance should be equal to the width of fs");
    vary_n_knobs(fs, center_inst, n, 0, out, end);
}


/**
 * Generates instances at distance n from the exemplar
 * (i.e., with n elements changed from 0 from the exemplar)
 * It calls a recursive function vary_n_knobs which varies
 * instance fields one by one (in all possible ways)
 *
 * @param fs   deme
 * @param n    distance
 * @param out  deme (where to store the instances)
 * @param end  deme iterator, containing the end iterator,
 *             necessary to check that 'out' does not go out
 *             of the deme if it does so then an assert is raised
 */
template<typename Out>
void generate_all_in_neighborhood(const eda::field_set& fs,
                                  unsigned n, Out out, Out end)
{
    eda::instance inst(fs.packed_width());
    generate_all_in_neighborhood(fs, n, out, end, inst);
}

/**
 * Used by the function generate_all_in_neighborhood (only) for
 * generating instances at distance n from a given instance considered
 * as center. It varies all possible n knobs in all possible ways. It
 * varies one instance field (at the changing position starting_index
 * and calls itself for the remaining fields).
 *
 * @todo: onto is ignored for the moment.
 *
 * @param fs              deme
 * @param inst            exemplar
 * @param n               distance
 * @param starting_index  position of a field to be varied
 * @param out             deme iterator (where to store the instances)
 * @param end             deme iterator, containing the end iterator,
 *                        necessary to check that 'out' does not go out
 *                        of the deme if it does so then an assert is raised
 * @return the out iterator pointing to the element after the last insertion
 */
template<typename Out>
Out vary_n_knobs(const eda::field_set& fs,
                 const eda::instance& inst,
                 unsigned n,
                 unsigned starting_index,
                 Out out, Out end)
{
    if(n == 0) {
        OC_ASSERT(out != end);  // to avoid invalid memory write
        *out++ = inst;
        return out;
    }

    eda::instance tmp_inst = inst;
    unsigned begin_contin_idx, begin_disc_idx, begin_bit_idx;

    // ontos
    if(starting_index < (begin_contin_idx = fs.n_onto())) {
        // @todo: handle onto
        out = vary_n_knobs(fs, tmp_inst, n, starting_index + begin_contin_idx,
                           out, end);
    }
    // contins
    else if(starting_index < (begin_disc_idx = begin_contin_idx + fs.n_contin())) {
        // modify the contin disc pointed by itr and recursive call
        eda::field_set::contin_iterator itc = fs.begin_contin(tmp_inst);
        size_t contin_idx = fs.raw_to_contin_idx(starting_index);
        itc += contin_idx;
        size_t depth = fs.contin()[itc.idx()].depth;
        size_t num = fs.count_n_before_stop(tmp_inst, contin_idx);
        eda::field_set::disc_iterator itr = fs.begin_raw(tmp_inst);        
        itr += starting_index;
        size_t relative_raw_idx = starting_index - fs.contin_to_raw_idx(contin_idx);
        // case tmp_inst at itr is Stop
        if(*itr == eda::field_set::contin_spec::Stop) {
            // Assumption [1]: within the same contin, it is the first Stop
            // recursive call, moved to the next contin (or disc if no more contin)
            out = vary_n_knobs(fs, tmp_inst, n,
                               // below is to fulfill Assumption [1]
                               starting_index + depth - relative_raw_idx,
                               out, end);
            // modify with Left or Right
            *itr = eda::field_set::contin_spec::Left;
            out = vary_n_knobs(fs, tmp_inst, n - 1, starting_index + 1, out, end);
            *itr = eda::field_set::contin_spec::Right;
            out = vary_n_knobs(fs, tmp_inst, n - 1, starting_index + 1, out, end);
        } 
        // case tmp_inst at itr is Left or Right
        else {
            // recursive call, moved for one position
            out = vary_n_knobs(fs, tmp_inst, n, starting_index + 1, out, end);
            // Left<->Right
            *itr = eda::field_set::contin_spec::switchLR(*itr);
            out = vary_n_knobs(fs, tmp_inst, n - 1, starting_index + 1, out, end);
            // if the next Stop is not further from itr than the distance n
            // then turn the remaining discs to Stop
            unsigned remRLs = num - relative_raw_idx; // remaining non-Stop
                                                      // discs including
                                                      // the current one
            if(remRLs <= n) {
                for(; relative_raw_idx < num; num--, itr++) {
                    // Stop
                    *itr = eda::field_set::contin_spec::Stop;
                }
                out = vary_n_knobs(fs, tmp_inst, n - remRLs,
                                   // below is to fulfill Assumption [1]
                                   starting_index + depth - relative_raw_idx,
                                   out, end);
            }
        }
    }
    // discs
    else if(starting_index < (begin_bit_idx = begin_disc_idx + fs.n_disc())) {
        eda::field_set::disc_iterator itd = fs.begin_disc(tmp_inst);
        itd += starting_index - begin_disc_idx;
        eda::disc_t tmp_val = *itd;
        // recursive call, moved for one position
        out = vary_n_knobs(fs, tmp_inst, n, starting_index + 1, out, end);
        // modify the disc and recursive call, moved for one position
        for(unsigned i = 1; i <= itd.arity() - 1; i++) {
            // vary all legal values, the neighborhood should 
            // not equals to itself, so if it is same, set it to 0.
            if(static_cast<unsigned>(tmp_val) == i)
                *itd = 0;
            else
                *itd = i;
            out = vary_n_knobs(fs, tmp_inst, n - 1, starting_index + 1, out, end);
        }
    }
    // bits
    else if(starting_index < begin_bit_idx + fs.n_bits()) {
        eda::field_set::bit_iterator itb = fs.begin_bits(tmp_inst);
        itb += starting_index - begin_bit_idx;

        // recursive call, moved for one position
        out = vary_n_knobs(fs, tmp_inst, n, starting_index + 1, out, end);
        // modify tmp_inst at itb, changed to the opposite value
        *itb = !(*itb);

        // recursive call, moved for one position
        out = vary_n_knobs(fs, tmp_inst, n - 1, starting_index + 1, out, end);
    }
    return out;
}


/**
 * Used by the function count_n_changed_knobs (only) for counting
 * instances at distance n from an instance considered as center
 * (inst). It counts all possible n knobs changed in all possible
 * ways.
 * 
 * @param fs              deme
 * @param inst            instance to consider the distance from
 * @param n               distance
 * @param starting_index  position of a field to be varied
 * @param max_count       stop counting when above this value, that is
 *                        because this function can be computationally expensive.
 */
inline unsigned long long count_n_changed_knobs_from_index(const eda::field_set& fs,
                                                           const eda::instance& inst,
                                                           unsigned n,
                                                           unsigned starting_index,
                                                           unsigned long long max_count
                                                           = numeric_limits<unsigned long long>::max())
{
    if(n == 0)
        return 1;

    unsigned long long number_of_instances = 0;

    unsigned begin_contin_idx = fs.n_onto();
    unsigned begin_disc_idx = begin_contin_idx + fs.n_contin();
    unsigned begin_bit_idx = begin_disc_idx + fs.n_disc();
    unsigned end_bit_idx = begin_bit_idx + fs.n_bits();

    // ontos
    if(starting_index < begin_contin_idx) {
        // @todo: handle onto
        number_of_instances = 
            count_n_changed_knobs_from_index(fs, inst, n,
                                             starting_index + begin_contin_idx,
                                             max_count);
    }
    // contins
    else if(starting_index < begin_disc_idx) {
        eda::field_set::const_contin_iterator itc = fs.begin_contin(inst);
        size_t contin_idx = fs.raw_to_contin_idx(starting_index);
        itc += contin_idx;
        int depth = fs.contin()[itc.idx()].depth;
        int num = fs.count_n_before_stop(inst, contin_idx);
        // it restricts the starting_index to be at the start of each
        // contin, otherwise should not be needed anyway
        OC_ASSERT(starting_index - fs.contin_to_raw_idx(contin_idx) == 0);
        // calculate number_of_instances for each possible distance i
        // of the current contin
        for(int i = 0; i <= min((int)n, depth); i++) {
            // number of instances for that contin, at distance i
            unsigned cni = 0;
            // count combinations when Left or Right are switched and
            // added after Stop, where j represents the number of
            // Left or Right added after Stop
            for(int j = max(0, i-num); j <= min(i, depth-num); j++)
                cni += (unsigned)binomial_coefficient<double>(num, i-j)
                    * pow2(j);
            // count combinations when Left or Right are switched and
            // removed before Stop, where j represents the number of
            // removed Left or Right before Stop
            if(i <= num)
                for(int j = 1; j <= min(i, num); j++)
                    cni += (unsigned)binomial_coefficient<double>(num-j, i-j);
            // recursive call
            number_of_instances +=
                cni * count_n_changed_knobs_from_index(fs, inst, n-i,
                                                       starting_index + depth,
                                                       max_count);
            // stop prematurely if above max_count
            if(number_of_instances > max_count)
                return number_of_instances;
        }
    }
    // discs
    else if(starting_index < begin_bit_idx) {
        eda::field_set::const_disc_iterator itd = fs.begin_disc(inst);
        itd += starting_index - begin_disc_idx;
        // recursive call, moved for one position
        number_of_instances = 
            count_n_changed_knobs_from_index(fs, inst, n, 
                                             starting_index + 1, max_count);
        // stop prematurely if above max_count
        if(number_of_instances > max_count)
            return number_of_instances;
        // count all legal values of the knob
        number_of_instances += 
            (itd.arity() - 1) 
            * count_n_changed_knobs_from_index(fs, inst, n - 1,
                                               starting_index + 1, max_count);
    }
    // bits
    else if(starting_index < end_bit_idx) {
        // since bits have the same arity (2) and are the last there
        // is no need for recursive call
        unsigned rb = end_bit_idx - starting_index;
        if(n <= rb) {
            double noi_db = binomial_coefficient<double>(rb , n);
            try {
                number_of_instances = numeric_cast<unsigned long long>(noi_db);
            } catch(positive_overflow&) {
                number_of_instances = numeric_limits<unsigned long long>::max();
            }
        }
    }
    return number_of_instances;
}

/**
 * Counts instances at distance n from the initial instance
 * (i.e., with n elements changed from the initial instance)
 * It calls a recursive function count_n_changed_knobs_from_index
 * 
 * @param fs              deme
 * @param inst            initial instance
 * @param n               distance
 * @param max_count       stop counting when above this value, that is
 *                        because this function can be computationally expensive.
 */
inline unsigned long long count_n_changed_knobs(const eda::field_set& fs,
                                                const eda::instance& inst,
                                                unsigned n,
                                                unsigned long long max_count 
                                                = numeric_limits<unsigned long long>::max())
{
    return count_n_changed_knobs_from_index(fs, inst, n, 0, max_count);
}
// for backward compatibility, like above but with null instance
inline unsigned long long count_n_changed_knobs(const eda::field_set& fs,
                                                unsigned n,
                                                unsigned long long max_count
                                                = numeric_limits<unsigned long long>::max())
{
    eda::instance inst(fs.packed_width());
    return count_n_changed_knobs_from_index(fs, inst, n, 0, max_count);
}

} // ~namespace moses

#endif // _OPENCOG_NEIGHBORHOOD_SAMPLING_H
