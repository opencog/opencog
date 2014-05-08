/*
 * opencog/learning/moses/moses/moses_params.h
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
#ifndef _MOSES_MOSES_PARAMS_H
#define _MOSES_MOSES_PARAMS_H

#include <atomic>
#include <map>
#include <boost/program_options/variables_map.hpp>

namespace opencog {
namespace moses {

/// A map between hostname and number of jobs allocated.
typedef std::map<std::string, unsigned> jobs_t;

/**
 * parameters to decide how to run moses
 */
struct moses_parameters
{
    moses_parameters(const boost::program_options::variables_map& _vm =
                           boost::program_options::variables_map(),
                     const jobs_t& _jobs = jobs_t(),
                     bool _local = true,
                     int _max_evals = 10000,
                     int _max_gens = -1,
                     score_t _max_score = 0,
                     int _max_cnd_output = -1) :
        local(_local), mpi(false), force_feed(false), jobs(_jobs), vm(_vm),
        max_evals(_max_evals), max_gens(_max_gens), max_score(_max_score),
        max_time(INT_MAX), max_cnd_output(_max_cnd_output)
    {}

    // Distributed solver control.
    bool local;
    bool mpi;
    bool force_feed;
    /* const */ jobs_t jobs;
    /* const */ boost::program_options::variables_map vm;

    // total maximun number of evals
    int max_evals;
    
    // the max number of demes to create and optimize, if negative,
    // then no limit
    int max_gens;
    
    // the max score
    score_t max_score;

    // limit amount of time that process should run
    time_t max_time;

    // the maximum number of candidates to output (if negative then it
    // output all candidates
    int max_cnd_output;
};

/**
 * Keep track of miscellaneous solver statistics
 */
struct moses_statistics
{
    moses_statistics() : n_evals(0), n_expansions(0), elapsed_secs(0)
    {}

    // total number of scoring function evaluations
    std::atomic<int> n_evals;

    // total number of deme expansions (generations)
    std::atomic<int> n_expansions;

    // elapsed wall-clock time, in seconds, since start of moses.
    double elapsed_secs;
};


} // ~namespace moses
} // ~namespace opencog

#endif
