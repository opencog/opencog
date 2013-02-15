/** moses_main.cc ---
 *
 * Copyright (C) 2012 Pouling Holdings
 *
 * Author: Linas Vepstas <linasvepstas@gmailcom>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <opencog/util/Logger.h>

#include "../metapopulation/metapopulation.h"
#include "distributed_moses.h"
#include "local_moses.h"
#include "mpi_moses.h"

#include "moses_main.h"

namespace opencog { namespace moses {

#define strform(x) #x
#define stringify(x) strform(x)

#ifdef MOSES_GIT_DESCRIBE
const char * version_string =
    stringify(MOSES_VERSION_MAJOR) "."
    stringify(MOSES_VERSION_MINOR) "."
    stringify(MOSES_VERSION_PATCH) " (git-describe "
    stringify(MOSES_GIT_DESCRIBE) ")";

#else
const char * version_string =
    stringify(MOSES_VERSION_MAJOR) "."
    stringify(MOSES_VERSION_MINOR) "."
    stringify(MOSES_VERSION_PATCH);

#endif

void print_stats_header (optim_stats *os, bool diversity)
{
    // Print legend for the columns of the stats.
    if (logger().isInfoEnabled()) {
        stringstream ss;
        ss << "Stats: # \n"
           << "Stats: # Stats are tab-separated, ready for graphing.\n"
           << "Stats: # You can also use the script parse_log.py to extract a CSV file given a moses log file.\n"
           << "Column explanation:\n"
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

void run_moses(metapopulation& metapop,
               const moses_parameters& moses_params,
               moses_statistics& stats)
{
    // Run moses, either on localhost, or distributed.
    if (moses_params.local)
        local_moses(metapop, moses_params, stats);
    else if (moses_params.mpi)
        mpi_moses(metapop, moses_params, stats);
    else
        distributed_moses(metapop, moses_params, stats);
}


} // ~namespace moses
} // ~namespace opencog

