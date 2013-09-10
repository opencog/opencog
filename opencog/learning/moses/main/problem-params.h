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
#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/table/table.h>
#include <opencog/learning/moses/metapopulation/metapop_params.h>
#include <opencog/learning/moses/moses/moses_main.h>
#include <opencog/learning/moses/moses/moses_params.h>
#include <opencog/learning/moses/optimization/optimization.h>
#include <opencog/learning/moses/optimization/hill-climbing.h>


namespace opencog { namespace moses {

struct problem_params
{
    problem_params();
    void parse_options(int argc, char* argv[]);

    // program options, see options_description below for their meaning
    std::vector<std::string> jobs_str;
    unsigned min_pool;
    bool enable_mpi;

    unsigned long rand_seed;
    std::vector<std::string> input_data_files;
    std::string target_feature;
    std::string problem;

    // default number of samples to describe a problem
    const unsigned int default_nsamples;
    int nsamples;
    float min_rand_input;
    float max_rand_input;
    unsigned long max_evals;
    time_t max_time;
    long result_count;
    bool output_score;
    bool output_penalty;
    bool output_bscore;
    bool output_only_best;
    bool output_eval_number;
    bool output_with_labels;
    bool output_python;
    std::string output_file;
    int max_gens;
    std::string log_level;
    std::string log_file;
    bool log_file_dep_opt;
    float noise;
    std::vector<std::string> include_only_ops_str;
    std::vector<std::string> ignore_ops_str;
    vertex_set ignore_ops;
    std::vector<std::string> ignore_features_str;
    std::string opt_algo; //optimization algorithm
    std::vector<std::string> exemplars_str;
    std::vector<combo_tree> exemplars;
    int reduct_candidate_effort;
    int reduct_knob_building_effort;
    bool weighted_accuracy;

    // metapop_param
    int max_candidates;
    bool reduce_all;
    unsigned revisit;
    score_t complexity_temperature;
    score_t complexity_ratio;
    double cap_coef;
    unsigned cache_size;
    bool linear_regression;
    float perm_ratio;
    
    // diversity parameters
    bool include_dominated;
    score_t diversity_pressure;
    score_t diversity_exponent;
    bool diversity_normalize;
    std::string diversity_dst;
    score_t diversity_p_norm;
    std::string diversity_dst2dp;

    // optim_param
    double pop_size_ratio;
    score_t max_score;
    size_t max_dist;

    // contin optimization
    std::vector<contin_t> discretize_thresholds;

    // hardness of the activation range
    // constraint for problems pre, recall, prerec
    score_t hardness;

    // hc_param
    bool hc_widen_search;
    bool hc_single_step;
    bool hc_crossover;
    unsigned hc_crossover_pop_size;
    bool hc_allow_resize_deme;
    unsigned hc_max_nn;
    double   hc_frac_of_nn;

    // classifier paramters
    bool use_well_enough;

    // pre params
    bool pre_worst_norm;
    bool gen_best_tree;

    // it params
    bool it_abs_err;

    // XXX Demo options, these should be removed!
    // viz the demo problems should get access to argc, argv...
    // and do their own parsing.
    std::string combo_str;
    unsigned int problem_size;

    // interesting predicates options.
    // XXX just like above, the ip argv parser should grab these...
    double ip_kld_weight;
    double ip_skewness_weight;
    double ip_stdU_weight;
    double ip_skew_U_weight;

    /// Enable feature selection while selecting exemplar
    /// feature selection happens before each representation building
    bool enable_feature_selection;
    std::string fs_focus;
    std::string fs_seed;
    feature_selector_parameters festor_params;
    feature_selection_parameters& fs_params;

    reduct::rule* bool_reduct;
    reduct::rule* bool_reduct_rep;
    reduct::rule* contin_reduct;

    optim_parameters opt_params;
    hc_parameters hc_params;
    moses_parameters moses_params;
    metapop_parameters meta_params;
    metapop_printer mmr_pa;
protected:
    const unsigned int max_filename_size;

    reduct::logical_reduction lr;
    options_description desc;
    std::vector<std::string> col_labels;

    void options_init();
};

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MOSES_PROBLEM_PARAMS_H
