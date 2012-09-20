/*
 * opencog/learning/moses/optimization/optimization.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012 Poulin Holdings
 * All Rights Reserved
 *
 * Written by Moshe Looks
 *            Predrag Janicic
 *            Nil Geisweiller
 *            Xiaohui Liu
 *            Linas Vepstas
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
#include "optimization.h"

namespace opencog { namespace moses {

double
information_theoretic_bits(const field_set& fs)
{
    static double log_five = log2<double>(5.0);

    double res = 0;

    size_t n_disc_fields = fs.n_disc_fields();
    vector<field_set::disc_spec>::const_iterator it = fs.disc_and_bit().begin();
    for (size_t cnt=0; cnt < n_disc_fields; cnt++, it++) {
        const field_set::disc_spec& d = *it;
        res += log2<double>(d.multy);
    }

    res += fs.n_bits();  // log_2(2)==1
    res += fs.contin().size() * log_five;

    foreach (const field_set::term_spec& o, fs.term())
        res += log2<double>(o.branching) * double(o.depth);

    return res;
}

optim_parameters::optim_parameters(const string& _opt_algo,
                 double _pop_size_ratio,
                 score_t _terminate_if_gte,
                 size_t _max_dist,
                 score_t _min_score_improv) :
    opt_algo(_opt_algo),
    term_improv(1.0),

    window_size_pop(0.05), //window size for RTR is
    window_size_len(1),    //min(windowsize_pop*N,windowsize_len*n)

    pop_size_ratio(_pop_size_ratio),    
    terminate_if_gte(_terminate_if_gte),
    max_dist(_max_dist)
{
    set_min_score_improv(_min_score_improv);
}

// N = p.popsize_ratio * n^1.05
// XXX Why n^1.05 ??? This is going to have a significant effect
// (as compared to n^1.00) only when n is many thousands or bigger...
unsigned optim_parameters::pop_size(const field_set& fs)
{
    return ceil(pop_size_ratio *
                 pow(information_theoretic_bits(fs), 1.05));
}

// term_improv*sqrt(n/w)  Huh?
unsigned optim_parameters::max_gens_improv(const field_set& fs)
{
    return ceil(term_improv*
                sqrt(information_theoretic_bits(fs) /
                     rtr_window_size(fs)));
}

// min(windowsize_pop*N,windowsize_len*n)
unsigned optim_parameters::rtr_window_size(const field_set& fs)
{
    return ceil(min(window_size_pop*pop_size(fs),
                    window_size_len*information_theoretic_bits(fs)));
}

unsigned optim_parameters::max_distance(const field_set& fs)
{
    return std::min(max_dist, fs.dim_size());
}

/// The score must improve by at least 's' to be considered; else
/// the search is terminated.  If 's' is negative, then it is
/// interpreted as a fraction: so 's=0.05' means 'the score must
/// improve 5 percent'.  
void optim_parameters::set_min_score_improv(score_t s)
{
    min_score_improvement = s;
}

score_t optim_parameters::min_score_improv()
{
    return min_score_improvement;
}

bool optim_parameters::score_improved(score_t best_score, score_t prev_hi)
{
    bool big_step = false;
    score_t imp = min_score_improv();

    if (0.0 <= imp)
         big_step = (best_score >  prev_hi + imp);
    else {
         // Score has improved if it increased by 0.5, or if it
         // increased by |imp| percent.  One extra minus sign
         // because imp is negative...
         big_step = (best_score >  prev_hi - imp * fabs(prev_hi));
         // big_step | = (best_score >  prev_hi + 0.5);
    }

    return big_step;
}

deme_size_t hill_climbing::cross_top_one(instance_set<composite_score>& deme,
                          deme_size_t deme_size,
                          deme_size_t num_to_make,
                          deme_size_t sample_start,
                          deme_size_t sample_size,
                          const instance& base)
{
    if (sample_size-1 < num_to_make) num_to_make = sample_size-1;

    // We need to access the high-scorers.
    // We don't actually need them all sorted; we only need
    // the highest-scoring num_to_make+1 to appear first, and
    // after that, we don't care about the ordering.
    // std::sort(next(deme.begin(), sample_start),
    //          next(deme.begin(), sample_start + sample_size),
    //          std::greater<scored_instance<composite_score> >());
    std::partial_sort(next(deme.begin(), sample_start),
                      next(deme.begin(), sample_start + num_to_make+1),
                      next(deme.begin(), sample_start + sample_size),
                      std::greater<scored_instance<composite_score> >());

    deme.resize(deme_size + num_to_make);

    const instance &reference = deme[sample_start].first;

    // Skip the first entry; its the top scorer.
    for (unsigned i = 0; i< num_to_make; i++) {
        unsigned j = deme_size + i;
        deme[j] = deme[sample_start + i + 1]; // +1 to skip the first one
        deme.fields().merge_instance(deme[j], base, reference);
    }
    return num_to_make;
}

/** two-dimensional simplex version of above. */
deme_size_t hill_climbing::cross_top_two(instance_set<composite_score>& deme,
                          deme_size_t deme_size,
                          deme_size_t num_to_make,
                          deme_size_t sample_start,
                          deme_size_t sample_size,
                          const instance& base)
{
    // sample_size choose two.
    unsigned max = sample_size * (sample_size-1) / 2;
    if (max < num_to_make) num_to_make = max;

    // std::sort(next(deme.begin(), sample_start),
    //          next(deme.begin(), sample_start + sample_size),
    //          std::greater<scored_instance<composite_score> >());
    //
    unsigned num_to_sort = sqrtf(2*num_to_make) + 3;
    if (sample_size < num_to_sort) num_to_sort = sample_size;
    std::partial_sort(next(deme.begin(), sample_start),
                      next(deme.begin(), sample_start + num_to_sort),
                      next(deme.begin(), sample_start + sample_size),
                      std::greater<scored_instance<composite_score> >());

    deme.resize(deme_size + num_to_make);

    // Summation is over a 2-simplex
    for (unsigned i = 1; i < sample_size; i++) {
        const instance& reference = deme[sample_start+i].first;
        for (unsigned j = 0; j < i; j++) {
            unsigned n = i*(i-1)/2 + j;
            if (num_to_make <= n) return num_to_make;
            unsigned ntgt = deme_size + n;
            deme[ntgt] = deme[sample_start + j];
            deme.fields().merge_instance(deme[ntgt], base, reference);
        }
    }
    return num_to_make;
}

/** three-dimensional simplex version of above. */
deme_size_t hill_climbing::cross_top_three(instance_set<composite_score>& deme,
                          deme_size_t deme_size,
                          deme_size_t num_to_make,
                          deme_size_t sample_start,
                          deme_size_t sample_size,
                          const instance& base)
{
    // sample_size choose three.
    unsigned max = sample_size * (sample_size-1) * (sample_size-2) / 6;
    if (max < num_to_make) num_to_make = max;

    // std::sort(next(deme.begin(), sample_start),
    //           next(deme.begin(), sample_start + sample_size),
    //           std::greater<scored_instance<composite_score> >());

    unsigned num_to_sort = cbrtf(6*num_to_make) + 3;
    if (sample_size < num_to_sort) num_to_sort = sample_size;
    std::partial_sort(next(deme.begin(), sample_start),
                      next(deme.begin(), sample_start + num_to_make),
                      next(deme.begin(), sample_start + sample_size),
                      std::greater<scored_instance<composite_score> >());
    deme.resize(deme_size + num_to_make);

    // Summation is over a 3-simplex
    for (unsigned i = 2; i < sample_size; i++) {
        const instance& iref = deme[sample_start+i].first;
        for (unsigned j = 1; j < i; j++) {
            const instance& jref = deme[sample_start+j].first;
            for (unsigned k = 0; k < i; k++) {
                unsigned n = i*(i-1)*(i-2)/6 + j*(j-1)/2 + k;
                if (num_to_make <= n) return num_to_make;
                unsigned ntgt = deme_size + n;
                deme[ntgt] = deme[sample_start + k];
                deme.fields().merge_instance(deme[ntgt], base, iref);
                deme.fields().merge_instance(deme[ntgt], base, jref);
            }
        }
    }
    return num_to_make;
}

}}
