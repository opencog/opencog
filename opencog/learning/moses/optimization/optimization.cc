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

#include <math.h>   // for sqrt

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

    for (const field_set::term_spec& o : fs.term())
        res += log2<double>(o.branching) * double(o.depth);

    return res;
}

optim_parameters::optim_parameters(const std::string& _opt_algo,
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
unsigned optim_parameters::pop_size(const field_set& fs) const
{
    return ceil(pop_size_ratio *
                 pow(information_theoretic_bits(fs), 1.05));
}

// term_improv*sqrt(n/w)  Huh?
unsigned optim_parameters::max_gens_improv(const field_set& fs) const
{
    return ceil(term_improv*
                sqrt(information_theoretic_bits(fs) /
                     rtr_window_size(fs)));
}

// min(windowsize_pop*N,windowsize_len*n)
unsigned optim_parameters::rtr_window_size(const field_set& fs) const
{
    return std::ceil(std::min(window_size_pop*pop_size(fs),
                    window_size_len*information_theoretic_bits(fs)));
}

unsigned optim_parameters::max_distance(const field_set& fs) const
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

score_t optim_parameters::min_score_improv() const
{
    return min_score_improvement;
}

bool optim_parameters::score_improved(score_t best_score, score_t prev_hi) const
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

} // ~namespace moses
} // ~namespace opencog

