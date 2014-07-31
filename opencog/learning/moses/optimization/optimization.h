/*
 * opencog/learning/moses/optimization/optimization.h
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * Copyright (C) 2012 Poulin Holdings LLC
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
#ifndef _MOSES_OPTIMIZATION_H
#define _MOSES_OPTIMIZATION_H

#include <opencog/util/oc_assert.h>

#include "../representation/instance_scorer.h"
#include "../representation/instance_set.h"

namespace opencog { namespace moses {

/**
 * information_theoretic_bits -- return information content of the field set.
 *
 * The information content of a single variable is defined as log base
 * two of the number of possible values that the variable can take.
 * Thus, for example, a single bit has two possible values, and an
 * information content of exactly one.
 *
 * The information-theoretic content of a contin variable is log_2(5).
 * This is because there are five ways to alter a contin: twiddle the
 * least-significant pseudo-bit (2 choices: L,R), erase the least
 * significant bit (1 choice), add a new least-significant bit
 * (2 choices: L,R).
 *
 * This routine sums the total information content in a field set,
 * including that in contin and term fields, as well as the discrete
 * and boolean fields.
 *
 * This is usually a good estimate for the total number of nearest
 * neighbors of an instance, rarely differing by more than twenty
 * percent. This is because the nearest neighbors of an instance are
 * those that differ by a Hamming distance of one, and the total
 * length of the instance is approximately equal to info-theo-bits!
 */
double information_theoretic_bits(const field_set& fs);

// optimization algorithms
static const std::string un("un"); // univariate
static const std::string sa("sa"); // star-shaped search
static const std::string hc("hc"); // local search

// Parameters used mostly for EDA algorithms but also possibly by
// other algo
struct optim_parameters
{
    optim_parameters(const std::string& _opt_algo = hc,
                     double _pop_size_ratio = 20,
                     score_t _terminate_if_gte = 0,
                     size_t _max_dist = 4,
                     score_t _min_score_improv = 0.5);

    // N = p.popsize_ratio * n^1.05
    // XXX Why n^1.05 ??? This is going to have a significant effect
    // (as compared to n^1.00) only when n is many thousands or bigger...
    unsigned pop_size(const field_set& fs) const;

    // term_improv*sqrt(n/w)  Huh?
    unsigned max_gens_improv(const field_set& fs) const;

    // min(windowsize_pop*N,windowsize_len*n)
    unsigned rtr_window_size(const field_set& fs) const;

    unsigned max_distance(const field_set& fs) const;

    /// The score must improve by at least 's' to be considered; else
    /// the search is terminated.  If 's' is negative, then it is
    /// interpreted as a fraction: so 's=0.05' means 'the score must
    /// improve 5 percent'.
    void set_min_score_improv(score_t s);

    score_t min_score_improv() const;

    bool score_improved(score_t best_score, score_t prev_hi) const;

    // String name of the optimization algo to employ
    std::string opt_algo;

    // optimization is terminated after n generations, or
    // term_improv*sqrt(n/w) consecutive generations with no
    // improvement (w=windowsize)
    double term_improv;

    double window_size_pop;
    double window_size_len;

    // Populations are sized at N = popsize_ratio*n^1.05 where n is
    // problem size in info-theoretic bits.  (XXX Why 1.05 ???)
    double pop_size_ratio;

    // Optimization is terminated if best score is >= terminate_if_gte
    score_t terminate_if_gte;

    // Defines the max distance to search during one iteration (used
    // in method max_distance)
    size_t max_dist;

private:
    // We accept improvement only if the score improved by that amount
    // or better, to avoid fine-tuning. It may be better to restart
    // the search from a new exemplar rather wasting time climbing a
    // near-plateau. Most problems have 1.0 as the smallest meaningful
    // score change, so 0.5 as default seems reasonable...
    score_t min_score_improvement;
};

// Enable some additional algorithm-dependent statistics gathering.
// Turned off by default because it's a CPU-waster.
// #define GATHER_STATS 1

/// Statistics obtained during optimization run, useful for tuning.
struct optim_stats
{
    optim_stats()
        : nsteps(0), demeID(0), total_steps(0), total_evals(0),
        field_set_size(0), over_budget(false)
#ifdef GATHER_STATS
          , hiscore(0.0), hicount(0.0),
          num_improved(0.0), count_improved(0.0)
#endif
    {}
    unsigned nsteps;
    demeID_t demeID;
    unsigned total_steps;
    unsigned total_evals;
    unsigned field_set_size;
    bool over_budget;

    // Additional stats.
#ifdef GATHER_STATS
    vector<double> scores;
    vector<double> counts;
    double hiscore;
    double hicount;
    double num_improved;
    double count_improved;
#endif
};

void print_stats_header (optim_stats *, bool diversity_enabled);

typedef instance_set<composite_score> deme_t;
typedef scored_instance<composite_score> deme_inst_t;

// Base class for all optimizers
struct optimizer_base : optim_stats
{
    optimizer_base(const optim_parameters& op = optim_parameters())
        : opt_params(op) {}
    
    virtual void operator()(deme_t& deme,
                            const iscorer_base& iscorer,
                            unsigned max_evals,
                            time_t max_time) = 0;

    virtual ~optimizer_base() {}

    optim_parameters opt_params;
};

} // ~namespace moses
} // ~namespace opencog

#endif
