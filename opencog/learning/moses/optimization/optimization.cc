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
#include <iostream>

#include "optimization.h"

namespace opencog { namespace moses {

double
information_theoretic_bits(const field_set& fs)
{
    static double log_five = log2<double>(5.0);

    double res = 0;

    size_t n_disc_fields = fs.n_disc_fields();
    std::vector<field_set::disc_spec>::const_iterator it = fs.disc_and_bit().begin();
    for (size_t cnt=0; cnt < n_disc_fields; ++cnt, ++it) {
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


void print_stats_header(optim_stats *os, bool diversity)
{
    using std::stringstream;

    // Print legend for the columns of the stats.
    if (logger().isInfoEnabled()) {
        stringstream ss;
        ss << "Stats: # \n"
           << "Stats: # Stats are tab-separated, ready for graphing.\n"
           << "Stats: # You can also use the script parse_log.py to extract a CSV file given a moses log file.\n"
           << "Stats: # Column explanation:\n"
           << "Stats: # \n"
           << "Stats: # gen is the generation number.\n"
           << "Stats: # num_evals is the number of scoring function evaluations so far.\n"
           << "Stats: # elapsed is the wall-clock time, in seconds, since start.\n"
           << "Stats: # metapop_size is the size of the metapopulation.\n"
           << "Stats: # best_score is the highest raw score seen, of all exemplars.\n"
           << "Stats: # complexity is in bits, of the highest-composite score exemplar.\n";
        if (os) {
           ss << "Stats: # field_set_size is the ESTIMATED number of bits in all the knobs.\n"
              << "Stats: # optim_steps is the number of steps the optimizer took.\n"
              << "Stats: # over_budget is bool, T if search exceeded scoring func eval budget.\n";
        }
        if (diversity) {
            ss << "Stats: # n_pairs is the number of pairs of candidates used to compute the diversity stats.\n"
               << "Stats: # mean_dst is the average bscore distance between 2 candidates.\n"
               << "Stats: # std_dst is the standard deviation of the average bscore distance between 2 candidates.\n"
               << "Stats: # min_dst is the minimum bscore distance between 2 candidates.\n"
               << "Stats: # max_dst is the maximum bscore distance between 2 candidates.\n";
            ss << "Stats: # best_n_pairs is the number of pairs of candidates used to compute the diversity stats amongst the best candidates to be output.\n"
               << "Stats: # best_mean_dst is the average bscore distance between 2 candidates amongst the best candidates to be output.\n"
               << "Stats: # best_std_dst is the standard deviation of the average bscore distance between 2 candidates amongst the best candidates to be output.\n"
               << "Stats: # best_min_dst is the minimum bscore distance between 2 candidates amongst the best candidates to be output.\n"
               << "Stats: # best_max_dst is the maximum bscore distance between 2 candidates amongst the best candidates to be output.\n";
        }
        ss << "Stats: # \n"
           << "Stats: # gen\tnum_evals\telapsed\tmetapop_size\tbest_score\tcomplexity";
        if (os) {
            ss << "\tfield_set_size\toptim_steps\tover_budget";
        }
        if (diversity) {
            ss << "\tn_pairs\tmean_dst\tstd_dst\tmin_dst\tmax_dst"
               << "\tbest_n_pairs\tbest_mean_dst\tbest_std_dst\tbest_min_dst\tbest_max_dst";                
        }
        logger().info(ss.str());
    }
}

} // ~namespace moses
} // ~namespace opencog

