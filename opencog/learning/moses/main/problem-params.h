/**
 * problem-params.h ---
 *
 * Copyright (C) 2013 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_MOSES_PROBLEM_PARAMS_H
#define _OPENCOG_MOSES_PROBLEM_PARAMS_H

#include <string>
#include <vector>

#include <boost/program_options.hpp>

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/table/table.h>
#include <opencog/learning/moses/main/problem.h>
#include <opencog/learning/moses/metapopulation/metapop_params.h>
#include <opencog/learning/moses/moses/moses_main.h>
#include <opencog/learning/moses/moses/moses_params.h>
#include <opencog/learning/moses/optimization/optimization.h>
#include <opencog/learning/moses/optimization/hill-climbing.h>


namespace opencog { namespace moses {

// XXX FIXME TODO The structure below should be split into multiple
// parts, with each sub-part responsible for picking out the argv's
// that it cares about. Unfortunately, this requires getting rid of 
// boost::program_options (because boost::program_options does not
// allow modulariztion in this way; it forces all program options to
// be specified in a single instance of options_description; if this
// isn't done, then parse_command_line() blows up, and notify() blows
// up. And then theres variables_map to hack around. So its badly
// designed.  I tried to hack past the damage but its overwhelming.
// Argh ...
struct problem_params : public option_base
{
    problem_params();
    void add_options(boost::program_options::options_description&);
    void parse_options(boost::program_options::variables_map&);

    // program options, see options_description below for their meaning
    std::vector<std::string> jobs_str;
    unsigned min_pool;
    bool enable_mpi;

    unsigned long rand_seed;
    std::string problem;

    // default number of samples to describe a problem
    const unsigned int default_nsamples;
    int nsamples;
    double min_rand_input;
    double max_rand_input;

    // Whether to balance the table
    bool balance;

    // Upper limit on resources spent
    unsigned long max_evals;
    time_t max_time;
    int max_gens;

    // Logger options
    std::string log_level;
    std::string log_file;

    // output printing options (metapop_printer)
    long result_count;
    bool output_score;
    bool output_cscore;
    bool output_bscore;
    bool output_only_best;
    bool output_eval_number;
    bool output_with_labels;
    bool output_deme_id;
    bool output_python;
    std::string output_file;

    // reduct options
    int reduct_candidate_effort;
    int reduct_knob_building_effort;
    std::vector<std::string> include_only_ops_str;
    std::vector<std::string> ignore_ops_str;
    vertex_set ignore_ops;
    std::vector<std::string> exemplars_str;
    std::vector<combo_tree> exemplars;

    // metapop_param
    int max_candidates;
    int revisit;
    bool reduce_all;
    bool linear_regression;
    bool discard_dominated;
    double noise;
    score_t complexity_temperature;
    score_t complexity_ratio;
    double cap_coef;
    unsigned cache_size;
    double perm_ratio;
    bool boosting;
    int num_to_promote;
    bool exact_experts;
    double expalpha;
    double bias_scale;
    
    // metapopulation diversity parameters
    score_t diversity_pressure;
    score_t diversity_exponent;
    bool diversity_normalize;
    std::string diversity_dst;
    score_t diversity_p_norm;
    std::string diversity_dst2dp;

    // optim_param (applicable for all optimzation algos)
    std::string opt_algo; //optimization algorithm
    double pop_size_ratio;
    score_t max_score;
    size_t max_dist;

    // contin optimization
    bool weighted_accuracy;
    std::vector<contin_t> discretize_thresholds;

    // Control temporal dispersion pressure, typically between 0 and 1
    score_t time_dispersion_pressure;

    // Control temporal dispersion pressure, distort the penalty
    score_t time_dispersion_exponent;

    // Spread bscore across timestamps instead of datapoints
    bool time_bscore;

    // Control granularity of time bscore (day or month)
    string time_bscore_granularity_str;
    TemporalGranularity time_bscore_granularity;

    // hc_param  (hill-climbing)
    bool hc_widen_search;
    bool hc_single_step;
    bool hc_crossover;
    unsigned hc_crossover_pop_size;
    unsigned hc_crossover_min_neighbors;
    bool hc_allow_resize_deme;
    unsigned hc_max_nn;
    double   hc_frac_of_nn;

    // classifier parameters
    bool use_well_enough;

    // hardness of the activation range -- scoring-related
    // constraint for problems pre, recall, prerec
    score_t hardness;

    // pre params
    bool pre_worst_norm;
    bool gen_best_tree;

    // it params
    bool it_abs_err;

    // Subsample deme params
    unsigned ss_n_subsample_demes,
        ss_n_top_candidates,
        ss_n_tuples,
        ss_n_best_bfdemes;
    float ss_std_dev_threshold,
        ss_tanimoto_mean_threshold,
        ss_tanimoto_geo_mean_threshold,
        ss_tanimoto_max_threshold,
        ss_tanimoto_mean_weight,
        ss_tanimoto_geo_mean_weight,
        ss_tanimoto_max_weight;
    bool ss_by_time,
        ss_contiguous_time;

    // Subsample fitness params
    unsigned ss_n_subsample_fitnesses;
    float ss_low_dev_pressure;
    
    /// Enable feature selection while selecting exemplar
    /// feature selection happens before each representation building
    bool enable_feature_selection;
    std::string fs_focus;
    std::string fs_seed;
    feature_selector_parameters festor_params;
    feature_selection_parameters& fs_params;
    std::string fs_enforce_features_filename;

    reduct::rule* bool_reduct;
    reduct::rule* bool_reduct_rep;
    reduct::rule* contin_reduct;

    optim_parameters opt_params;
    hc_parameters hc_params;
    moses_parameters moses_params;
    deme_parameters deme_params;
    subsample_deme_filter_parameters filter_params;
    metapop_parameters meta_params;
    metapop_printer mmr_pa;
protected:
    const unsigned int max_filename_size;

    reduct::logical_reduction lr;

private:
    std::vector<std::string> col_labels;
};

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MOSES_PROBLEM_PARAMS_H
