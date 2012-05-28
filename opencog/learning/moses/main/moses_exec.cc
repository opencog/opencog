/** moses_exec.cc ---
 *
 * Copyright (C) 2010 OpenCog Foundation
 *
 * Author: Nil Geisweiller <ngeiswei@gmail.com>
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
#include <boost/format.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <opencog/util/numeric.h>
#include <opencog/util/log_prog_name.h>

#include "moses_exec.h"
#include "../moses/partial.h"
#include "../example-progs/scoring_iterators.h"

namespace opencog { namespace moses {

using boost::format;
using boost::trim;
using boost::str;

static const unsigned int max_filename_size = 255;

/**
 * Display error message about unspecified combo tree and exit
 */
void unspecified_combo_exit()
{
    cerr << "Error: you must specify which combo tree to learn (option -y)."
         << endl;
    exit(1);
}

/**
 * Display error message about unsupported type and exit
 */
void unsupported_type_exit(const type_tree& tt)
{
    cerr << "Error: type " << tt << " currently not supported." << endl;
    exit(1);
}
void unsupported_type_exit(type_node type)
{
    unsupported_type_exit(type_tree(type));
}

/**
 * Display error message about ill formed combo tree and exit
 */
void illformed_exit(const combo_tree& tr) {
    cerr << "Error: apparently the combo tree "
         << tr << "is not well formed." << endl;
    exit(1);
}

/**
 * Display error message about unsupported problem and exit
 */
void unsupported_problem_exit(const string& problem) {
    cerr << "Error: problem " << problem
         << " unsupported for the moment." << endl;
    exit(1);
}

/**
 * Display error message about missing input data file and exit
 */
void no_input_datafile_exit() {
    cerr << "No input data file has been specified (option -"
         << input_data_file_opt.second << ")" << endl;
    exit(1);
}

/**
  * Display error that not all data files have same arity and exit
  */
void not_all_same_arity_exit(const string& input_data_file1, arity_t arity1,
                             const string& input_data_file2, arity_t arity2)
{
    cerr << "Error: File " << input_data_file1 << " has arity " << arity1
         << " while file " << input_data_file2 << "has_arity "
         << arity2 << endl;
    exit(1);
}

/**
 * Display error message about not recognized combo operator and exist
 */
void not_recognized_combo_operator(const string& ops_str) {
    cerr << "Error: " << ops_str
         << " is not recognized as combo operator." << endl;
    exit(1);
}

/**
 * determine the initial exemplar of a given type
 */
combo_tree type_to_exemplar(type_node type)
{
    switch(type) {
    case id::boolean_type: return combo_tree(id::logical_and);
    case id::contin_type: return combo_tree(id::plus);
    case id::enum_type: {
        combo_tree tr(id::cond);
        tr.append_child(tr.begin(), enum_t::get_random_enum());
        return tr;
    }
    case id::ill_formed_type:
        cerr << "Error: The data type is incorrect, perhaps it has not been"
             << " possible to infer it from the input table." << endl;
        exit(1);
    default:
        unsupported_type_exit(type);
    }
    return combo_tree();
}

combo_tree ann_exemplar(combo::arity_t arity)
{
    combo_tree ann_tr(ann_type(0, id::ann));
    // ann root
    combo_tree::iterator root_node = ann_tr.begin();
    // output node
    combo_tree::iterator output_node =
        ann_tr.append_child(root_node, ann_type(1, id::ann_node));
    // input nodes
    for (combo::arity_t i = 0; i <= arity; ++i)
        ann_tr.append_child(output_node, ann_type(i + 2, id::ann_input));
    // input nodes' weights
    ann_tr.append_children(output_node, 0.0, arity + 1);
 
    return ann_tr;
}

static bool contains_type(const type_tree_pre_it it, id::type_node ty)
{
    for (type_tree_sib_it tts = it.begin(); tts != it.end(); tts++) 
        if (*tts == ty) return true;
    return false;
}

/**
 * Determine the alphabet size given the type_tree of the problem and
 * a list of operators to ignore.  This is a somewhat ad-hoc value,
 * meant to help compute the complexity ratio.
 */
unsigned alphabet_size(const type_tree& tt, const vertex_set ignore_ops)
{
    unsigned as = 0;
    // arity will be zero for anything that isn't a lambda_type.
    // However, all tables will be lambda_type ...
    combo::arity_t arity = type_tree_arity(tt);
    type_node output_type = get_type_node(type_tree_output_type_tree(tt));

    switch (output_type) {
        case id::boolean_type:
            // 3 operators: and, or, not
            as = 3 + arity;
            break;

        case id::contin_type:
            // Set alphabet size, 8 is roughly the number of operators
            // in contin formula, it will have to be adapted
            // Well, there could be a mix of booleans too.
            as = 8 + arity - ignore_ops.size();
            break;

        case id::enum_type: {

            // Try to take into account a table with both booleans and
            // contins.
            type_tree_pre_it ttl = tt.begin();

            as = arity + enum_t::size();

            if (contains_type(ttl, id::contin_type))
                as += 8 - ignore_ops.size();

            if (contains_type(ttl, id::boolean_type)) 
                as += 3;

            break;
        }
        case id::ann_type:

            as = 2 + arity*arity; // to account for hidden neurons, very roughly
            break;

        case id::lambda_type:
        case id::application_type:
        case id::union_type:
        case id::arg_list_type:
        case id::action_result_type:
        case id::definite_object_type:
        case id::action_definite_object_type:
        case id::indefinite_object_type:
        case id::message_type:
        case id::action_symbol_type:
        case id::wild_card_type:
        case id::unknown_type:
        case id::ill_formed_type:
        default:
            unsupported_type_exit(tt);
    }

    logger().info() << "Alphabet size = " << as
                    << " output = " << output_type;
    return as;
}

// set the complexity ratio.
template <typename BScorer>
void set_noise_or_ratio(BScorer& scorer, unsigned as, float noise, score_t ratio)
{
    if (noise > 0.0)
        scorer.set_complexity_coef(as, noise);
    else
        scorer.set_complexity_coef(ratio);
}

//* Convert string to a combo_tree
combo_tree str_to_combo_tree(const string& combo_str)
{
    stringstream ss;
    combo_tree tr;
    ss << combo_str;
    ss >> tr;
    return tr;
}

//* Get largest contin constant in a combo tree
contin_t largest_const_in_tree(const combo_tree &tr)
{
    contin_t rc = 0.0;
    combo_tree::pre_order_iterator it;
    for(it = tr.begin(); it != tr.end(); it++) {
        if (is_contin(*it)) {
            contin_t val = get_contin(*it);
            if (rc < val) rc = val;
        }
    }

    return rc;
}

//* return true iff the problem is based on data file
bool datafile_based_problem(const string& problem)
{
    return problem == it || problem == pre || problem == ann_it || problem == ip;
}

//* return true iff the problem is based on a combo tree
bool combo_based_problem(const string& problem)
{
    return problem == cp || problem == ann_cp;
}

// Infer the arity of the problem
combo::arity_t infer_arity(const string& problem,
                           unsigned int problem_size,
                           const vector<string>& input_table_files,
                           const string& combo_str)
{
    if (datafile_based_problem(problem)) {
        if (input_table_files.empty())
            no_input_datafile_exit();

        combo::arity_t arity = dataFileArity(input_table_files.front());
        if (input_table_files.size() > 1) {
            // check that all input data files have the same arity
            combo::arity_t test_arity;
            for (size_t i = 1; i < input_table_files.size(); ++i) {
                test_arity = dataFileArity(input_table_files[i]);
                if (test_arity != arity) {
                    not_all_same_arity_exit(input_table_files[0], arity,
                                            input_table_files[i], test_arity);
                    return -1;
                }
            }
        }
        return arity;
    }
    else if (combo_based_problem(problem))
    {
        if (combo_str.empty())
            unspecified_combo_exit();
        // get the combo_tree and infer its type
        combo_tree tr = str_to_combo_tree(combo_str);
        type_tree tt = infer_type_tree(tr);
        if (is_well_formed(tt)) {
            return type_tree_arity(tt);
        } else {
            illformed_exit(tr);
            return -1;
        }
    }
    else if (problem == pa || problem == dj)
    {
        return problem_size;
    }
    else if (problem == mux)
    {
        return problem_size + (1<<problem_size);
    }
    else if (problem == sr)
    {
        return 1;
    }
    else
    {
        unsupported_problem_exit(problem);
        return -1;
    }
}

int moses_exec(int argc, char** argv)
{
    // for(int i = 0; i < argc; ++i)
    //     cout << "arg = " << argv[i] << endl;

    // program options, see options_description below for their meaning
    unsigned long rand_seed;
    vector<string> input_data_files;
    string target_feature;
    string problem;
    string combo_str;
    unsigned int problem_size;
    int nsamples;
    float min_rand_input;
    float max_rand_input;
    unsigned long max_evals;
    long result_count;
    bool output_score;
    bool output_complexity;
    bool output_bscore;
    bool output_dominated = false;
    bool output_eval_number;
    bool output_with_labels;
    bool output_python = false;
    string output_file;
    int max_gens;
    string log_level;
    string log_file;
    bool log_file_dep_opt;
    float noise;
    vector<string> include_only_ops_str;
    vector<string> ignore_ops_str;
    string opt_algo; //optimization algorithm
    vector<string> exemplars_str;
    int reduct_candidate_effort;
    int reduct_knob_building_effort;
    vector<string> jobs_str;
    bool weighted_accuracy;

    // metapop_param
    int max_candidates;
    bool reduce_all;
    bool revisit = false;
    bool include_dominated;
    score_t complexity_temperature = 5.0f;
    score_t complexity_ratio = 3.5f;
    bool enable_cache;

    // optim_param
    double pop_size_ratio;
    score_t max_score;
    size_t max_dist;
    // continuous optimization
    vector<contin_t> discretize_thresholds;
    double ip_kld_weight;
    double ip_skewness_weight;
    double ip_stdU_weight;
    double ip_skew_U_weight;
    score_t alpha;              // weight of the activation range
                                // constraint for problem pre
    // hc_param
    bool hc_widen_search;
    bool hc_single_step;
    bool hc_crossover;

    // pre params
    bool pre_worst_norm;

    // it params
    bool it_abs_err;

    // Declare the supported options.
    // XXX TODO: make this print correctly, instead of using brackets.
    options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Produce help message.\n")

        ("version", "Display the version of moses.\n")

        (opt_desc_str(rand_seed_opt).c_str(),
         value<unsigned long>(&rand_seed)->default_value(1),
         "Random seed.\n")

        (opt_desc_str(max_evals_opt).c_str(),
         value<unsigned long>(&max_evals)->default_value(10000),
         "Maximum number of fitness function evaluations.\n")

        (opt_desc_str(result_count_opt).c_str(),
         value<long>(&result_count)->default_value(10),
         "The number of results to return, ordered according to "
         "a linear combination of score and complexity. If negative, "
         "then return all results.\n")

        (opt_desc_str(output_score_opt).c_str(),
         value<bool>(&output_score)->default_value(true),
         "If 1, output the score before each candidate (at the left of the complexity).\n")

        (opt_desc_str(output_complexity_opt).c_str(),
         value<bool>(&output_complexity)->default_value(false),
         "If 1, output the complexity and the scoring penalty before each candidate (after the score).\n")

        (opt_desc_str(output_bscore_opt).c_str(),
         value<bool>(&output_bscore)->default_value(false),
         "If 1, output the bscore below each candidate.\n")

        (opt_desc_str(output_dominated_opt).c_str(),
         value<bool>(&output_dominated)->default_value(false),
         "If 1, print the the entire metapopulation, and not just the "
         "highest scoring candidates.\n")

        (opt_desc_str(output_eval_number_opt).c_str(),
         value<bool>(&output_eval_number)->default_value(false),
         "If 1, output the actual number of evaluations.\n")

        (opt_desc_str(output_with_labels_opt).c_str(),
         value<bool>(&output_with_labels)->default_value(false),
         "If 1, output the candidates with using the argument labels "
         "instead of argument numbers. For instance "
         "*(\"$price\" \"$temprature\") instead of *($1 $2). This only "
         "works for data fitting problems where the data file contains "
         "labels in its header.\n")

        ("python",
         value<bool>(&output_python)->default_value(false),
         "If 1, output the program(s) as python code instead of combo. "
         "Best with -c1 option to return a single python module. Only "
         "implemented for boolean programs currently.\n")

        (opt_desc_str(output_file_opt).c_str(),
         value<string>(&output_file)->default_value(""),
         "File where to place the output. If empty, then output to stdout.\n")

        (opt_desc_str(max_gens_opt).c_str(),
         value<int>(&max_gens)->default_value(-1),
         "Maximum number of demes to generate and optimize, negative means no generation limit.\n")

        (opt_desc_str(input_data_file_opt).c_str(),
         value<vector<string> >(&input_data_files),
         "Input table file in DSV format (with comma, whitespace and tabulation as seperator). Colums correspond to features and rows to observations. Can be used several times, in such a case the behavioral score of the whole problem is the concatenation of the behavioral scores of the sub-problems associated with the files. Each file must have the same number of features in the same order.\n")

        (opt_desc_str(target_feature_opt).c_str(),
         value<string>(&target_feature),
         "Label of the target feature to fit. If none is given the first one is used.\n")

        (opt_desc_str(problem_opt).c_str(),
         value<string>(&problem)->default_value(it),
         str(format("Problem to solve, supported problems are:\n"
                    "%s, regression based on input table\n"
                    "%s, regression based on input table but maximizing precision instead of accuracy\n"
                    "%s, search interesting patterns, where interestingness"
                    " is defined in terms of several features such as maximizing"
                    " the Kullback-Leibler"
                    " divergence between the distribution of the outputs and"
                    " that same distribution in the context of the pattern"
                    " being true."
                    " Or the difference of skewnesses between the 2 distributions"
                    " and other things being experimented.\n"
                    "%s, regression based on input table using ann\n"
                    "%s, regression based on combo program\n"
                    "%s, even parity\n"
                    "%s, disjunction\n"
                    "%s, multiplex\n"
                    "%s, regression of f(x)_o = sum_{i={1,o}} x^i\n")
             % it % pre % ip % ann_it % cp % pa % dj % mux % sr).c_str())

        (opt_desc_str(combo_str_opt).c_str(),
         value<string>(&combo_str),
         str(format("Combo program to learn, used when the problem"
                    " %s is selected (option -%s).\n")
             % cp % problem_opt.second).c_str())

        (opt_desc_str(problem_size_opt).c_str(),
         value<unsigned int>(&problem_size)->default_value(5),
         str(format("For even parity (%s), disjunction (%s) and multiplex (%s)"
                    " the problem size corresponds to the arity."
                    " For regression of f(x)_o = sum_{i={1,o}} x^i (%s)"
                    " the problem size corresponds to the order o.\n")
             % pa % dj % mux % sr).c_str())

        (opt_desc_str(nsamples_opt).c_str(),
         value<int>(&nsamples)->default_value(-1),
         "Number of samples to describe the problem. "
         "If nsample is negative, null or larger than the maximum "
         "number of samples allowed it is ignored. If the default "
         "problem size is larger than the value provided with that "
         "option then the dataset is subsampled randomly to reach the "
         "target size.\n")

        (opt_desc_str(min_rand_input_opt).c_str(),
         value<float>(&min_rand_input)->default_value(0.0),
         "Minimum value of a sampled coninuous input.  The cp, ip, and "
         "pre problems all require a range of values to be sampled in "
         "order to measure the fitness of a proposed solution. This "
         "option sets the low end of the sampled range. In the case of "
         "fitness function pre, the range corresponds to the activation "
         "of the precision.\n")

        (opt_desc_str(max_rand_input_opt).c_str(),
         value<float>(&max_rand_input)->default_value(1.0),
         "Maximum value of a sampled coninuous input.  The cp, ip, and "
         "pre problems all require a range of values to be sampled in "
         "order to measure the fitness of a proposed solution. This "
         "option sets the low high of the sampled range. In the case of "
         "fitness function pre, the range corresponds to the activation "
         "of the precision.\n")

        (opt_desc_str(log_level_opt).c_str(),
         value<string>(&log_level)->default_value("INFO"),
         "Log level, possible levels are NONE, ERROR, WARN, INFO, "
         "DEBUG, FINE. Case does not matter.\n")

        (opt_desc_str(log_file_dep_opt_opt).c_str(),
         str(format("The name of the log is determined by the options, for"
                    " instance if moses-exec is called with -%s 123 -%s %s"
                    " the log name is moses_random-seed_123_problem_pa.log."
                    " The name will be truncated in order not to"
                    " be longer than %s characters.\n")
             % rand_seed_opt.second % problem_opt.second % pa
             % max_filename_size).c_str())

        (opt_desc_str(log_file_opt).c_str(),
         value<string>(&log_file)->default_value(default_log_file),
         str(format("File name where to write the log."
                    " This option is overwritten by %s.\n")
             % log_file_dep_opt_opt.first).c_str())

        (opt_desc_str(include_only_ops_str_opt).c_str(),
         value<vector<string> >(&include_only_ops_str),
         "Include this operator, but exclude others, in the solution.  "
         "This option may be used several times to specify multiple "
         "operators.  Currently, only these operators are "
         "supported: plus, times, div, sin, exp, log and variables ($n). "
         "Note that variables and operators are treated separately, so "
         "that including only some operators will still include all "
         "variables, and including only some variables still include "
         "all operators).  You may need to put variables under double "
         "quotes.  This option does not work with ANN.\n")

        (opt_desc_str(ignore_ops_str_opt).c_str(),
         value<vector<string> >(&ignore_ops_str),
         str(format("Ignore the following operator in the program solution.  "
                    "This option may be used several times.  Currently, only div, "
                    "sin, exp, log  and variables ($n) can be ignored.  "
                    "You may need to put variables under double quotes.  "
                    "This option has the priority over --%s.  "
                    "That is, if an operator is both be included and ignored, "
                    "then it is ignored.  This option does not work with ANN.\n")
             % include_only_ops_str_opt.first).c_str())

        (opt_desc_str(opt_algo_opt).c_str(),
         value<string>(&opt_algo)->default_value(hc),
         str(format("Optimization algorithm, supported algorithms are"
                    " univariate (%s), simulation annealing (%s),"
                    " hillclimbing (%s).\n")
             % un % sa % hc).c_str())

        (opt_desc_str(exemplars_str_opt).c_str(),
         value<vector<string> >(&exemplars_str),
         "Start the search with a given exemplar, can be used several times.\n")

        (opt_desc_str(max_candidates_opt).c_str(),
         value<int>(&max_candidates)->default_value(-1),
         "Maximum number of considered candidates to be added to the metapopulation after optimizing deme.\n")

        (opt_desc_str(reduce_all_opt).c_str(),
         value<bool>(&reduce_all)->default_value(true),
         "Reduce all candidates before being evaluated.  Otherwise "
         "they are only reduced before being added to the "
         "metapopulation. This option can be valuable if memoization "
         "is enabled to avoid re-evaluate of duplicates.\n")

        (opt_desc_str(reduct_candidate_effort_opt).c_str(),
         value<int>(&reduct_candidate_effort)->default_value(2),
         "Effort allocated for reduction of candidates, in the range 0-3. "
         "0 means minimum effort, 3 means maximum effort.\n")

        (opt_desc_str(reduct_knob_building_effort_opt).c_str(),
         value<int>(&reduct_knob_building_effort)->default_value(2),
         "Effort allocated for reduction during knob building, 0-3, 0 means minimum effort, 3 means maximum effort. The bigger the effort the lower the dimension of the deme.\n")

        (opt_desc_str(enable_cache_opt).c_str(),
         value<bool>(&enable_cache)->default_value(false),
         "Memoize, that is, cache evaluation results, so that identical "
         "candidates are not re-evaluated. The cache size is dynamically "
         "adjusted to fit in the RAM.\n")

        (opt_desc_str(jobs_opt).c_str(),
         value<vector<string> >(&jobs_str),
         str(format("Number of jobs allocated for deme optimization."
                    " Jobs can be executed on a remote machine as well,"
                    " in such case the notation -%1% N:REMOTE_HOST is used,"
                    " where N is the number of jobs on the machine REMOTE_HOST."
                    " For instance one can enter the options"
                    " -%1%4 -%1%16%2%my_server.org"
                    " (or -%1%16%2%user@my_server.org if one wishes to"
                    " run the remote jobs under a different user name),"
                    " meaning that 4 jobs are allocated on the local machine"
                    " and 16 jobs are allocated on my_server.org."
                    " The assumption is that moses must be on the remote"
                    " machine and is located in a directory included in the"
                    " PATH environment variable. Beware that a lot of log"
                    " files are gonna be generated when using this option on"
                    " the remote machines.\n")
             % jobs_opt.second % job_seperator).c_str())

        (opt_desc_str(weighted_accuracy_opt).c_str(),
         value<bool>(&weighted_accuracy)->default_value(false),
         "This option is useful in case of unbalanced data as it "
         "weights the score so that each class weights equally "
         "regardless of their proportion in terms of sample size.\n")

        (opt_desc_str(pop_size_ratio_opt).c_str(),
         value<double>(&pop_size_ratio)->default_value(20),
         "The higher the more effort is spent on a deme.\n")

        (opt_desc_str(max_score_opt).c_str(),
         value<score_t>(&max_score)->default_value(best_score),
         "The max score to reach, once reached MOSES halts. MOSES is sometimes able to calculate the max score that can be reached for a particular problem, in such case the max_score is automatically reset of the minimum between MOSES's calculation and the user's option.\n")

        (opt_desc_str(max_dist_opt).c_str(),
         value<size_t>(&max_dist)->default_value(4),
         "The maximum radius of the neighborhood around the "
         "exemplar to explore.\n")

        (opt_desc_str(include_dominated_opt).c_str(),
         value<bool>(&include_dominated)->default_value(true),
         "Include dominated candidates (according behavioral score) "
         "when merging candidates in the metapopulation. Disabling "
         "this may lead to poorer performance.\n")

        (opt_desc_str(complexity_temperature_opt).c_str(),
         value<score_t>(&complexity_temperature)->default_value(6.0),
         "Set the \"temperature\" of the Boltzmann-like distribution "
         "used to select the next exemplar out of the metapopulaton. "
         "A temperature that is too high or too low will make it likely "
         "that poor exemplars will be chosen for exploration, thus "
         "resulting in excessively long search times.\n")

        (opt_desc_str(complexity_ratio_opt).c_str(),
         value<score_t>(&complexity_ratio)->default_value(3.5),
         "Fix the ratio of the score to complexity, to be used as a "
         "penalty, when ranking the metapopulation for fitness.  "
         "The complexity penalty is "
         "the inverse of the complexity ratio.  Setting this ratio "
         "too low (complexity penalty too high) causes the complexity "
         "to dominate ranking, probably trapping the algorithm in a "
         "local maximum.  Setting this ratio too high (complexity "
         "penalty too low) will waste time exploring unproductive "
         "solutions, adversely lengthening solution times.  "
         "Suggest setting this to a value that is 1x to 2x larger than "
         "the ratio of change in complexity to score improvement (as "
         "determined by earlier runs). \n")

        (opt_desc_str(noise_opt).c_str(),
         value<float>(&noise)->default_value(0),
         "Alternative way to set the ratio of raw score to complexity.  "
         "Setting this option over-rides the complexity ratio, above.  "
         "Assumes that the data is noisy.   The noisier the data, the "
         "stronger the model complexity penalty.  If the target feature "
         "is discrete, the setting should correspond to the fraction of "
         "the input data that might be wrong (i.e. the probability p "
         "that an output datum (row) is wrong).   In this case, only "
         "values 0 < p < 0.5 are meaningful (i.e. less than half the "
         "data can be wrong). Suggested values are in the range 0.01 to "
         "0.001.  If the target feature is continuous, the value sepcified "
         "should correspond to the standard deviation of the (Gaussian) "
         "noise centered around each candidate's output. A value of zero "
         "or less cedes this setting to complexity-ratio flag, above.\n")

        (opt_desc_str(discretize_threshold_opt).c_str(),
         value<vector<contin_t> >(&discretize_thresholds),
         "If the domain is continuous, discretize the target feature. A unique used of that option produces 2 classes, x < thresold and x >= threshold. The option can be used several times (n-1) to produce n classes and the thresholds are automatically sorted.\n")

        (opt_desc_str(hc_widen_search_opt).c_str(),
         value<bool>(&hc_widen_search)->default_value(false),
         str(format("Hillclimbing parameter (%s). If false, then deme search "
                    "terminates when a local hilltop is found. If true, "
                    "then the search radius is progressively widened, "
                    "until another termination condition is met.\n") % hc).c_str())

        (opt_desc_str(hc_single_step_opt).c_str(),
         value<bool>(&hc_single_step)->default_value(false),
         str(format("Hillclimbing parameter (%s). If false, then the normal "
                    "hillclimbing algorithm is used.  If true, then only one "
                    "step is taken towards the hilltop, and the results are "
                    "promptly folded back into the metapopulation. If this "
                    "flag is set, then consider using the widen-search flag "
                    "as well, so as to make forward progress.\n") % hc).c_str())

        (opt_desc_str(hc_crossover_opt).c_str(),
         value<bool>(&hc_crossover)->default_value(false),
         str(format("Hillclimbing parameter (%s). If false, then the entire "
                    "local neighborhood of the current center "
                    "instance is explored. The highest-scoring "
                    "instance is then chosen as the new center "
                    "instance, and the process is repeated.  For "
                    "many datasets, however, the highest-scoring "
                    "instances tend to cluster together, and so an "
                    "exhaustive search may not be required. When "
                    "this option is specified, a handful of the "
                    "highest-scoring instances are crossed-over (in "
                    "the genetic sense of cross-over) to create new "
                    "instances.  Only these are evaluated for "
                    "fitness; the exhaustive search step is skipped. "
                    "For many problem types, especialy those with "
                    "large neighborhoods (i.e. those with high "
                    "prorgram complexity), this can lead to an "
                    "order-of-magnitude speedup, or more.  For other "
                    "problem types, especially those with deceptive "
                    "scoring functions, this can hurt performance.\n"
                    ) % hc).c_str())

        (opt_desc_str(ip_kld_weight_opt).c_str(),
         value<double>(&ip_kld_weight)->default_value(1.0),
         str(format("Interesting patterns (%s). Weight of the KLD.\n") % ip).c_str())

        (opt_desc_str(ip_skewness_weight_opt).c_str(),
         value<double>(&ip_skewness_weight)->default_value(1.0),
         str(format("Interesting patterns (%s). Weight of skewness.\n") % ip).c_str())

        (opt_desc_str(ip_stdU_weight_opt).c_str(),
         value<double>(&ip_stdU_weight)->default_value(1.0),
         str(format("Interesting patterns (%s). Weight of stdU.\n") % ip).c_str())

        (opt_desc_str(ip_skew_U_weight_opt).c_str(),
         value<double>(&ip_skew_U_weight)->default_value(1.0),
         str(format("Interesting patterns (%s). Weight of skew_U.\n") % ip).c_str()) 

        (opt_desc_str(alpha_opt).c_str(),
         value<score_t>(&alpha)->default_value(0.0),
         "If problem pre is used then if alpha is negative (any negative value), precision is replaced by negative predictive value. And then alpha plays the role of the activation constrain penalty from 0 to inf, 0 being no activation penalty at all, inf meaning hard constraint penalty (that is if the candidate is not in the range it has -inf activation penalty.)\n")

        ("pre-worst-norm",
         value<bool>(&pre_worst_norm)->default_value(false),
         "Normalize the precision w.r.t. its worst decile [EXPERIMENTAL].\n")

        ("it-abs-err",
         value<bool>(&it_abs_err)->default_value(false),
         "Use absolute error instead of squared error [EXPERIMENTAL, the occam's razor hasn't been calibrated for that fitness function yet].\n")
       ;

    variables_map vm;
    try {
        store(parse_command_line(argc, argv, desc), vm);
    }
    catch (error& e) {
        OC_ASSERT(0, "Fatal error: invalid or duplicated argument:\n\t%s\n", e.what());
    }
    notify(vm);

    // set flags
    log_file_dep_opt = vm.count(log_file_dep_opt_opt.first) > 0;
 
    if (vm.count("help") || argc == 1) {
        cout << desc << endl;
        return 1;
    }

    if (vm.count("version")) {
        cout << "moses " << MOSES_VERSION_MAJOR
             << "." << MOSES_VERSION_MINOR
             << "." << MOSES_VERSION_PATCH
             << " (revno " << MOSES_BZR_REVNO << ")" << std::endl;;
        return 1;
    }

    // Set log file.
    if (log_file_dep_opt) {
        set<string> ignore_opt{log_file_dep_opt_opt.first};
        log_file = determine_log_name(default_log_file_prefix,
                                      vm, ignore_opt,
                                      string(".").append(default_log_file_suffix));
    }

    // Remove old log_file before setting the new one.
    remove(log_file.c_str());
    logger().setFilename(log_file);
    trim(log_level);
    Logger::Level level = logger().getLevelFromString(log_level);
    if (level != Logger::BAD_LEVEL)
        logger().setLevel(level);
    else {
        cerr << "Error: Log level " << log_level << " is incorrect (see --help)." << endl;
        exit(1);
    }
    logger().setBackTraceLevel(Logger::ERROR);

    // Log command-line args
    string cmdline = "Command line:";
    for (int i = 0; i < argc; ++i) {
         cmdline += " ";
         cmdline += argv[i];
    }
    logger().info(cmdline);

    // Init random generator.
    randGen().seed(rand_seed);

    // Infer arity
    combo::arity_t arity = infer_arity(problem, problem_size,
                                       input_data_files, combo_str);
    logger().info("Inferred arity = %d", arity);

    // Convert include_only_ops_str to the set of actual operators to
    // ignore.
    vertex_set ignore_ops;
    if (vm.count(include_only_ops_str_opt.first.c_str())) {
        bool ignore_arguments = false;
        bool ignore_operators = false;
        foreach (const string& s, include_only_ops_str) {
            vertex v;
            if (builtin_str_to_vertex(s, v)) {
                if (!ignore_operators) {
                    ignore_ops = {id::plus, id::times, id::div,
                                  id::exp, id::log, id::sin, id::impulse};
                    ignore_operators = true;
                }
                ignore_ops.erase(v);
            } else if (argument_str_to_vertex(s, v)) {
                if (!ignore_arguments) {
                    for (combo::arity_t arg = 1; arg <= arity; ++arg)
                        ignore_ops.insert(argument(arg));
                    ignore_arguments = true;
                }
                ignore_ops.erase(v);
            } else not_recognized_combo_operator(s);
        }
    }

    // Convert ignore_ops_str to the set of actual operators to ignore.
    foreach (const string& s, ignore_ops_str) {
        vertex v;
        if(builtin_str_to_vertex(s, v) || argument_str_to_vertex(s, v))
            ignore_ops.insert(v);
        else not_recognized_combo_operator(s);
    }

    // Set the initial exemplars.
    vector<combo_tree> exemplars;
    foreach(const string& exemplar_str, exemplars_str) {
        exemplars.push_back(str_to_combo_tree(exemplar_str));
    }

    // Fill jobs
    jobs_t jobs{{localhost, 1}}; // by default the localhost has 1 job
    bool only_local = true;
    foreach(const string& js, jobs_str) {
        size_t pos = js.find(job_seperator);
        if (pos != string::npos) {
            unsigned int nj = boost::lexical_cast<unsigned int>(js.substr(0, pos));
            string host_name = js.substr(pos + 1);
            jobs[host_name] = nj;
            only_local = false;
        } else {
            jobs[localhost] = boost::lexical_cast<unsigned int>(js);
        }
    }

    setting_omp(jobs[localhost]);
    
    // Set metapopulation parameters.
    metapop_parameters meta_params(max_candidates, reduce_all,
                                   revisit, include_dominated, 
                                   complexity_temperature,
                                   enable_cache,
                                   jobs[localhost]);

    // Set optim_parameters.
    optim_parameters opt_params(opt_algo, pop_size_ratio, max_score, max_dist);
    opt_params.hc_params.widen_search = hc_widen_search;
    opt_params.hc_params.single_step = hc_single_step;
    opt_params.hc_params.crossover = hc_crossover;

    // Set moses_parameters.
    moses_parameters moses_params(
        vm, jobs, only_local,
        max_evals, max_gens, max_score, ignore_ops);

    // Find the column number of the target feature in the data file,
    // if any.
    int target_column = 0;
    if (!target_feature.empty() && !input_data_files.empty())
        target_column = findTargetFeaturePosition(input_data_files.front(),
                                               target_feature);
    logger().info("Target column is %d", target_column);

    // Read labels contained in the data file.
    vector<string> labels;
    if (output_with_labels && !input_data_files.empty())
        labels = readInputLabels(input_data_files.front(), target_column);

    // Set metapop printer parameters.
    metapop_printer mmr_pa(result_count,
                           output_score,
                           output_complexity,
                           output_bscore,
                           output_dominated,
                           output_eval_number,
                           output_with_labels, 
                           labels,
                           output_file,
                           output_python);

    // Continuous reduction rules used during search and representation
    // building.
    const rule& contin_reduct = contin_reduction(reduct_candidate_effort, ignore_ops);

    // Logical reduction rules used during search.
    logical_reduction r(ignore_ops);
    const rule& bool_reduct = r(reduct_candidate_effort);

    // Logical reduction rules used during representation building.
    const rule& bool_reduct_rep = r(reduct_knob_building_effort);

    // Problem based on input table.
    if (datafile_based_problem(problem)) {

        // Infer the signature based on the input table.
        type_tree table_type_signature = infer_data_type_tree(input_data_files.front(), target_column);
        logger().info() << "Inferred data signature " << table_type_signature;

        // Read input data files
        vector<Table> tables;
        vector<CTable> ctables;
        foreach (const string& idf, input_data_files) {
            logger().debug("Read data file %s", idf.c_str());
            Table table = istreamTable(idf, target_column);
            // possible subsample the table
            if (nsamples > 0)
                subsampleTable(table, nsamples);
            tables.push_back(table);
            ctables.push_back(table.compress());
        }

        // 'it' means regression based on input table.
        // 'pre' means we must maximize precision (that is, negative
        // predictive value), instead of accuracy.
        if (problem == it || problem == pre) {

            // Infer the type of the input table
            type_tree table_output_tt = type_tree_output_type_tree(table_type_signature);
            type_node table_output_tn = get_type_node(table_output_tt);

            // Determine the default exemplar to start with
            // problem == pre  precision-based scoring
            if (exemplars.empty())
                exemplars.push_back(type_to_exemplar(
                    problem == pre? id::boolean_type : table_output_tn));

            type_node output_type =
                get_type_node(get_output_type_tree(*exemplars.begin()->begin()));
            if (output_type == id::unknown_type)
                output_type = table_output_tn;

            logger().info() << "Inferred output type: " << output_type;
            
            // problem == pre  precision-based scoring
            if (problem == pre) {
                type_tree cand_tt = gen_signature(id::boolean_type, arity);
                int as = alphabet_size(cand_tt, ignore_ops);
                typedef precision_bscore BScore;
                boost::ptr_vector<BScore> bscores;
                foreach(const CTable& ctable, ctables) {
                    BScore* r = new BScore(ctable,
                                           min_rand_input,
                                           max_rand_input,
                                           abs(alpha), alpha >= 0,
                                           pre_worst_norm);
                    set_noise_or_ratio(*r, as, noise, complexity_ratio);
                    bscores.push_back(r);
                }
                multibscore_based_bscore<BScore> bscore(bscores);
                metapop_moses_results(exemplars, cand_tt,
                                      bool_reduct, bool_reduct_rep, bscore,
                                      opt_params, meta_params, moses_params,
                                      mmr_pa);
            } 

            // problem == it  i.e. input-table based scoring.
            else {
                OC_ASSERT(output_type == table_output_tn);
                int as = alphabet_size(table_type_signature, ignore_ops);

                // --------- Boolean output type
                if (output_type == id::boolean_type) {
                    typedef ctruth_table_bscore BScore;
                    boost::ptr_vector<BScore> bscores;
                    foreach(const CTable& ctable, ctables) {
                        BScore *r = new BScore(ctable);
                        set_noise_or_ratio(*r, as, noise, complexity_ratio);
                        bscores.push_back(r);
                    }
                    multibscore_based_bscore<BScore> bscore(bscores);
                    metapop_moses_results(exemplars, table_type_signature,
                                          bool_reduct, bool_reduct_rep, bscore,
                                          opt_params, meta_params, moses_params,
                                          mmr_pa);
                }

                // --------- Enumerated output type
                else if (output_type == id::enum_type) {
#if 1
                    // This is ifdef'd out, replaced by the
                    // "leave well-enough alone" algorithm. It gives
                    // a hint at the original, but dumb, implementation.
                    //
                    // Much like the boolean type above, just a
                    // slightly different scorer.
                    typedef enum_graded_bscore BScore;
                    boost::ptr_vector<BScore> bscores;
                    foreach(const CTable& ctable, ctables) {
                        BScore *r = new BScore(ctable);
                        set_noise_or_ratio(*r, as, noise, complexity_ratio);
                        bscores.push_back(r);
                    }
                    multibscore_based_bscore<BScore> bscore(bscores);
                    metapop_moses_results(exemplars, table_type_signature,
                               contin_reduct, contin_reduct, bscore,
                               opt_params, meta_params, moses_params,
                               mmr_pa);

#else
                    partial_solver well(ctables,
                                        table_type_signature,
                                        exemplars, contin_reduct,
                                        opt_params, meta_params,
                                        moses_params, mmr_pa);
                    well.solve();
#endif
                }

                // --------- Contin output type
                else if (output_type == id::contin_type) {
                    if (discretize_thresholds.empty()) {
                        typedef contin_bscore BScore;
                        boost::ptr_vector<BScore> bscores;
                        contin_bscore::err_function_type eft =
                            it_abs_err ? contin_bscore::abs_error :
                            contin_bscore::squared_error;
                        foreach(const Table& table, tables) {
                            BScore *r = new BScore(table, eft);
                            set_noise_or_ratio(*r, as, noise, complexity_ratio);
                            bscores.push_back(r);
                        }
                        multibscore_based_bscore<BScore> bscore(bscores);
                        metapop_moses_results(exemplars, table_type_signature,
                                              contin_reduct, contin_reduct, bscore,
                                              opt_params, meta_params, moses_params,
                                              mmr_pa);
                    } else {
                        typedef discretize_contin_bscore BScore;
                        boost::ptr_vector<BScore> bscores;
                        foreach(const Table& table, tables) {
                            BScore *r = new BScore(table.otable, table.itable,
                                                   discretize_thresholds,
                                                   weighted_accuracy);
                            set_noise_or_ratio(*r, as, noise, complexity_ratio);
                            bscores.push_back(r);
                        }
                        multibscore_based_bscore<BScore> bscore(bscores);
                        metapop_moses_results(exemplars, table_type_signature,
                                              contin_reduct, contin_reduct, bscore,
                                              opt_params, meta_params, moses_params,
                                              mmr_pa);
                    }
                }

                // --------- Unknown output type
                else {
                    unsupported_type_exit(output_type);
                }
            }
        }
        
        // Find interesting patterns
        else if (problem == ip) {
            // ip assumes that the inputs are boolean and the output is contin
            type_tree ettt = gen_signature(id::boolean_type,
                                           id::contin_type, arity);
            OC_ASSERT(ettt == table_type_signature,
                      "The input table doesn't have the right data types."
                      " The output should be contin and the inputs should"
                      " be boolean");
            // signature of the functions to learn
            type_tree tt = gen_signature(id::boolean_type, arity);

            // determine the default exemplar to start with
            if (exemplars.empty())
                exemplars.push_back(type_to_exemplar(id::boolean_type));

            int as = alphabet_size(tt, ignore_ops);

            typedef interesting_predicate_bscore BScore;
            boost::ptr_vector<BScore> bscores;
            foreach(const CTable& ctable, ctables) {
                BScore *r = new BScore(ctable,
                                       ip_kld_weight,
                                       ip_skewness_weight,
                                       ip_stdU_weight,
                                       ip_skew_U_weight,
                                       min_rand_input,
                                       max_rand_input,
                                       alpha, alpha >= 0);
                set_noise_or_ratio(*r, as, noise, complexity_ratio);
                bscores.push_back(r);
            }
            multibscore_based_bscore<BScore> bscore(bscores);
            metapop_moses_results(exemplars, tt,
                                  bool_reduct, bool_reduct_rep, bscore,
                                  opt_params, meta_params, moses_params,
                                  mmr_pa);
        }

        // regression based on input table using ann
        else if (problem == ann_it)
        {

            // If no exemplar has been provided in the options,
            // insert the default.
            if (exemplars.empty()) {
                exemplars.push_back(ann_exemplar(arity));
            }

            type_tree tt = gen_signature(id::ann_type, 0);

            int as = alphabet_size(tt, ignore_ops);

            contin_bscore bscore(tables.front());
            set_noise_or_ratio(bscore, as, noise, complexity_ratio);
            metapop_moses_results(exemplars, tt,
                                  ann_reduction(), ann_reduction(), bscore,
                                  opt_params, meta_params, moses_params, mmr_pa);
        }
    }

    // Demo/Example: Problem based on input combo program.
    // Learn a program that should be identical to the specified input
    // program.
    else if (combo_based_problem(problem))
    {
        combo_tree tr = str_to_combo_tree(combo_str);

        // If the user specifies the combo program from bash or similar
        // shells, and forgets to escape the $ in the variable names,
        // then the resulting combo program will be garbage.  Try to
        // sanity-check this, so as to avoid user frustration. 
        // A symptom of this error is that the arity will be -1.
        if (-1 == arity || NULL == strchr(combo_str.c_str(), '$')) {
            cerr << "Error: the combo program " << tr << "\n"
                 << "appears not to contain any arguments. Did you\n"
                 << "forget to escape the $'s in the shell command line?"
                 << endl;
            exit(2);
        }

        if (problem == cp) { // Regression based on combo program
            // Get the combo_tree and infer its type
            type_tree tt = infer_type_tree(tr);

            type_node output_type = get_type_node(type_tree_output_type_tree(tt));

            // If no exemplar has been provided in the options, use the
            // default one.
            if (exemplars.empty()) {
                exemplars.push_back(type_to_exemplar(output_type));
            }

            if (output_type == id::boolean_type) {
                // @todo: Occam's razor and nsamples is not taken into account
                logical_bscore bscore(tr, arity);
                metapop_moses_results(exemplars, tt,
                                      bool_reduct, bool_reduct_rep, bscore,
                                      opt_params, meta_params, moses_params,
                                      mmr_pa);
            }
            else if (output_type == id::contin_type) {

                // Naive users of the combo regression mode will fail
                // to understand that the input program must be sampled
                // in order for the fitness function to be evaluated.
                // The default sample range 0<x<1 is probably too small
                // for any fancy-pants input program, so try to make
                // a reasonable guess.  Yes, this is a stupid hack, but
                // it does avoid the problem of naive users saying
                // "aww moses sucks" when they fail to invoke it correctly.
                if ((0.0 == min_rand_input) && (1.0 == max_rand_input)) {
                    max_rand_input = 2.0 * largest_const_in_tree(tr);
                    min_rand_input = -max_rand_input;
                    if ((nsamples <= 0) && 
                        (default_nsamples < 2 * arity * max_rand_input)) {
                        nsamples = 2 * arity * max_rand_input;
                    }
                }

                if (nsamples <= 0)
                    nsamples = default_nsamples;

                logger().info() << "Will sample combo program " << tr << "\n"
                                << "\tat " << nsamples << " input values, "
                                << "ranging between " << min_rand_input
                                << " and " << max_rand_input <<endl;

                // @todo: introduce some noise optionally
                ITable it(tt, nsamples, max_rand_input, min_rand_input);
                OTable ot(tr, it);

                int as = alphabet_size(tt, ignore_ops);

                contin_bscore bscore(ot, it);
                set_noise_or_ratio(bscore, as, noise, complexity_ratio);
                metapop_moses_results(exemplars, tt,
                                      contin_reduct, contin_reduct, bscore,
                                      opt_params, meta_params, moses_params,
                                      mmr_pa);
            } else {
                unsupported_type_exit(tt);
            }
        }
        else if (problem == ann_cp)
        {
            // regression based on combo program using ann
            // get the combo_tree and infer its type
            type_tree tt = gen_signature(id::ann_type, 0);
            int as = alphabet_size(tt, ignore_ops);

            // If no exemplar has been provided in the options,
            // use the default.
            if (exemplars.empty()) {
                exemplars.push_back(ann_exemplar(arity));
            }
 
            // @todo: introduce some noise optionally
            if (nsamples <= 0)
                nsamples = default_nsamples;

            ITable it(tt, nsamples, max_rand_input, min_rand_input);
            OTable ot(tr, it);
 
            contin_bscore bscore(ot, it);
            set_noise_or_ratio(bscore, as, noise, complexity_ratio);
            metapop_moses_results(exemplars, tt,
                                  contin_reduct, contin_reduct, bscore,
                                  opt_params, meta_params, moses_params, mmr_pa);
        }
    }

    // Demo/Example: learn a combo program that determines if the
    // program inputs are even parity or not.  That is, the combo
    // program will be a boolean circuit that computes parity.
    else if (problem == pa)
    {
        even_parity func;

        // If no exemplar has been provided in the options, use the
        // default boolean_type exemplar (which is 'and').
        if (exemplars.empty()) {
            exemplars.push_back(type_to_exemplar(id::boolean_type));
        }

        type_tree sig = gen_signature(id::boolean_type, arity);
        unsigned as = alphabet_size(sig, ignore_ops); 

        logical_bscore bscore(func, arity);
        set_noise_or_ratio(bscore, as, noise, complexity_ratio);

        metapop_moses_results(exemplars, sig,
                              bool_reduct, bool_reduct_rep, bscore,
                              opt_params, meta_params, moses_params, mmr_pa);
    }

    // Demo/example problem: learn the logical disjunction. That is,
    // moses should learn the following program: or($1 $2 ... $k) where
    // k is the number of inputs specified by the -k option.
    else if (problem == dj)
    {
        // @todo: for the moment occam's razor and partial truth table are ignored
        disjunction func;

        // If no exemplar has been provided in the options, use the
        // default boolean_type exemplar (which is 'and').
        if (exemplars.empty()) {
            exemplars.push_back(type_to_exemplar(id::boolean_type));
        }

        type_tree tt = gen_signature(id::boolean_type, arity);
        logical_bscore bscore(func, arity);
        metapop_moses_results(exemplars, tt,
                              bool_reduct, bool_reduct_rep, bscore,
                              opt_params, meta_params, moses_params, mmr_pa);
    }

    // Demo/example problem: multiplex. Learn the combo program that
    // corresponds to the boolean (electrical) circuit that is a
    // (de-)multiplexer.  That is, a k-bit binary address will specify
    // one and exactly one wire out of 2^k wires.  Here, k==problem_size.
    else if (problem == mux)
    {
        // @todo: for the moment occam's razor and partial truth table are ignored
        // arity = problem_size + 1<<problem_size
        multiplex func(problem_size);

        // If no exemplar has been provided in the options, use the
        // default boolean_type exemplar (which is 'and').
        if (exemplars.empty()) {
            exemplars.push_back(type_to_exemplar(id::boolean_type));
        }

        type_tree tt = gen_signature(id::boolean_type, arity);
        logical_bscore bscore(func, arity);
        metapop_moses_results(exemplars, tt,
                              bool_reduct, bool_reduct_rep, bscore,
                              opt_params, meta_params, moses_params, mmr_pa);
    }

    // Demo/Example problem: polynomial regression.  Given the polynomial
    // p(x)=x+x^2+x^3+...x^k, this searches for the  shortest  program
    // consisting  of  nested arithmetic operators to compute p(x),
    // given x as a free variable.  So, for example the order-2 polynomial
    // can be written as x+x^2, and the shortest combo program is
    // *(+(1 $1) $1) (that is, the  solution is p(x)=x(x+1) in the usual
    // arithmetical notation).
    else if (problem == sr)
    { // simple regression of f(x)_o = sum_{i={1,o}} x^i

        // If no exemplar has been provided in the options, use the
        // default contin_type exemplar (+)
        if (exemplars.empty()) {
            exemplars.push_back(type_to_exemplar(id::contin_type));
        }
 
        type_tree tt = gen_signature(id::contin_type, arity);

        ITable it(tt, (nsamples>0 ? nsamples : default_nsamples));

        int as = alphabet_size(tt, ignore_ops);
        
        contin_bscore::err_function_type eft =
            it_abs_err ? contin_bscore::abs_error :
            contin_bscore::squared_error;
        contin_bscore bscore(simple_symbolic_regression(problem_size),
                             it, eft);
        set_noise_or_ratio(bscore, as, noise, complexity_ratio);
        metapop_moses_results(exemplars, tt,
                              contin_reduct, contin_reduct, bscore,
                              opt_params, meta_params, moses_params, mmr_pa);
    }
    else
       unsupported_problem_exit(problem);
    return 0;
}

int moses_exec(const vector<string>& argvs)
{
    char** argv = new char*[argvs.size()];
    for(size_t i = 0; i < argvs.size(); ++i) {
        argv[i] = const_cast<char*>(argvs[i].c_str());
    }
    int res = moses_exec(argvs.size(), argv);
    delete[] argv;
    return res;
}

} // ~namespace moses
} // ~namespace opencog
