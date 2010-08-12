/** moses_options_names.h --- 
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


#ifndef _OPENCOG_MOSES_OPTIONS_NAMES_H
#define _OPENCOG_MOSES_OPTIONS_NAMES_H

#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>

// number of evals string
static const string number_of_evals_str = "#evals";

// program option names and abbreviations
static const string rand_seed_opt_name = "random-seed";
static const string rand_seed_opt_ab = "r";

static const string input_table_file_opt_name = "input-file";
static const string input_table_file_opt_ab = "i";

static const string problem_opt_name = "problem";
static const string problem_opt_ab = "H";

static const string combo_str_opt_name = "combo-program";
static const string combo_str_opt_ab = "y";

static const string problem_size_opt_name = "problem-size";
static const string problem_size_opt_ab = "k";

static const string nsamples_opt_name = "nsamples";
static const string nsamples_opt_ab = "b";

static const string min_rand_input_opt_name = "min-rand-input";
static const string min_rand_input_opt_ab = "q";

static const string max_rand_input_opt_name = "max-rand-input";
static const string max_rand_input_opt_ab = "w";

static const string max_evals_opt_name = "max-evals";
static const string max_evals_opt_ab = "m";

static const string result_count_opt_name = "result-count";
static const string result_count_opt_ab = "c";

static const string output_bscore_opt_name = "output-bscore";
static const string output_bscore_opt_ab = "t";

static const string output_complexity_opt_name = "output-complexity";
static const string output_complexity_opt_ab = "x";

static const string output_eval_number_opt_name = "output-eval-number";
static const string output_eval_number_opt_ab = "V";

static const string max_gens_opt_name = "max-gens";
static const string max_gens_opt_ab = "g";

static const string log_level_opt_name = "log-level";
static const string log_level_opt_ab = "l";

static const string log_file_opt_name = "log-file";
static const string log_file_opt_ab = "f";
static const string default_log_file_prefix = "moses";
static const string default_log_file_suffix = "log";
static const string default_log_file = default_log_file_prefix + "." + default_log_file_suffix;

static const string log_file_dep_opt_opt_name = "log-file-dep-opt";
static const string log_file_dep_opt_opt_ab = "L";

static const string variance_opt_name = "variance";
static const string variance_opt_ab = "v";

static const string prob_opt_name = "probability";
static const string prob_opt_ab = "p";

static const string ignore_ops_str_opt_name = "ignore-operator";
static const string ignore_ops_str_opt_ab = "n";

static const string opt_algo_opt_name = "opt-algo";
static const string opt_algo_opt_ab = "a";

static const string exemplars_str_opt_name = "exemplar";
static const string exemplars_str_opt_ab = "e";

static const string reduct_candidate_effort_opt_name = "reduct-candidate-effort";
static const string reduct_candidate_effort_opt_ab = "E";

static const string reduct_knob_building_effort_opt_name = "reduct-knob-building-effort";
static const string reduct_knob_building_effort_opt_ab = "B";

static const string reduce_all_opt_name = "reduce-all";
static const string reduce_all_opt_ab = "d";


static const string count_base_opt_name = "count-based-scorer";
static const string count_base_opt_ab = "u";

static const string cache_size_opt_name = "cache-size";
static const string cache_size_opt_ab = "s";

static const string revisit_opt_name = "revisit";
static const string revisit_opt_ab = "R";

static const string jobs_opt_name = "jobs";
static const string jobs_opt_ab = "j";

static const string pop_size_ratio_opt_name = "pop-size-ratio";
static const string pop_size_ratio_opt_ab = "P";

// used to convert program option argument to string.
// @todo: very ugly, it is likely something better can be done using
// boost::program_options API
template<typename T>
bool to_string(const boost::program_options::variable_value& vv, string& str)
{
    if(vv.value().type() == typeid(T)) {
        str = boost::lexical_cast<string>(vv.as<T>());
        return true;
    }
    return false;
}
string to_string(const boost::program_options::variable_value& vv)
{
    string res;
    if(to_string<int>(vv, res))
        return res;
    else if(to_string<unsigned int>(vv, res))
        return res;
    else if(to_string<long>(vv, res))
        return res;
    else if(to_string<unsigned long>(vv, res))
        return res;
    else if(to_string<float>(vv, res))
        return res;
    else if(to_string<double>(vv, res))
        return res;
    else if(to_string<bool>(vv, res))
        return res;
    else if(to_string<string>(vv, res))
        return res;
    else {
        std::cerr << "type not handled yet" << std::endl;
        return res;
    }
}

#endif // _OPENCOG_MOSES_OPTIONS_NAMES_H
