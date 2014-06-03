/** moses_exec_def.h --- 
 *
 * Copyright (C) 2010 Novamente LLC
 *
 * Author: Nil Geisweiller
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

#ifndef _OPENCOG_MOSES_EXEC_DEF_H
#define _OPENCOG_MOSES_EXEC_DEF_H

#include <string>

namespace opencog { namespace moses {

using namespace std;

// program option names and abbreviations (one letter). If the
// abbreviation is empty there is none. For their meanings see
// options_description in moses-exec.cc.
//
// Available abbreviations are: O
static const pair<string, string> rand_seed_opt("random-seed", "r");
static const pair<string, string> problem_opt("problem", "H");
static const pair<string, string> nsamples_opt("nsamples", "b");
static const pair<string, string> min_rand_input_opt("min-rand-input", "q");
static const pair<string, string> max_rand_input_opt("max-rand-input", "w");
static const pair<string, string> max_evals_opt("max-evals", "m");
static const pair<string, string> result_count_opt("result-count", "c");
static const pair<string, string> output_score_opt("output-score", "S");
static const pair<string, string> output_penalty_opt("output-penalty", "x");
static const pair<string, string> output_bscore_opt("output-bscore", "t");
static const pair<string, string> output_only_best_opt("output-only-best", "C");
static const pair<string, string> output_eval_number_opt("output-eval-number", "V");
static const pair<string, string> output_with_labels_opt("output-with-labels", "W");
static const pair<string, string> output_file_opt("output-file", "o");
static const pair<string, string> max_gens_opt("max-gens", "g");
static const pair<string, string> log_level_opt("log-level", "l");
static const pair<string, string> log_file_opt("log-file", "f");
static const string default_log_file_prefix = "moses";
static const string default_log_file_suffix = "log";
static const string default_log_file = default_log_file_prefix + "." + default_log_file_suffix;
static const pair<string, string> log_file_dep_opt_opt("log-file-dep-opt", "F");
static const pair<string, string> noise_opt("noise", "p");
static const pair<string, string> include_only_ops_str_opt("include-only-operator", "N");
static const pair<string, string> ignore_ops_str_opt("ignore-operator", "n");
static const pair<string, string> opt_algo_opt("opt-algo", "a");
static const pair<string, string> exemplars_str_opt("exemplar", "e");
static const pair<string, string> reduct_candidate_effort_opt("reduct-candidate-effort", "E");
static const pair<string, string> reduct_knob_building_effort_opt("reduct-knob-building-effort", "B");
static const pair<string, string> reduce_all_opt("reduce-all", "d");
static const pair<string, string> cache_size_opt("cache-size", "s");
static const pair<string, string> jobs_opt("jobs", "j");
static const string job_seperator(":");
static const string localhost("localhost");
static const pair<string, string> weighted_accuracy_opt("weighted-accuracy", "G");
static const pair<string, string> pop_size_ratio_opt("pop-size-ratio", "P");
static const pair<string, string> max_score_opt("max-score", "A");
static const pair<string, string> max_dist_opt("max-dist", "D");
static const pair<string, string> max_candidates_opt("max-candidates", "M");
static const pair<string, string> include_dominated_opt("include-dominated", "I");
static const pair<string, string> complexity_temperature_opt("complexity-temperature", "v");
static const pair<string, string> complexity_ratio_opt("complexity-ratio", "z");
static const pair<string, string> discretize_threshold_opt("discretize-threshold", "R");

// options specific to a particular fitness function
static const pair<string, string> alpha_opt("alpha", "Q");

// options specific to a particular optimization algorithm
static const pair<string, string> hc_single_step_opt("hc-single-step", "L");
static const pair<string, string> hc_widen_search_opt("hc-widen-search", "T");
static const pair<string, string> hc_crossover_opt("hc-crossover", "Z");

// Returns a string interpretable by Boost.Program_options
// "name,abbreviation"
string opt_desc_str(const pair<string, string>& opt);

} // ~namespace moses
} // ~namespace opencog

#endif // _OPENCOG_MOSES_OPTIONS_NAMES_H
