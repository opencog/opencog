/** moses_exec.cc ---
 *
 * Copyright (C) 2010 OpenCog Foundation
 * Copyright (C) 2012 Poulin Holdings LLC
 *
 * Author: Nil Geisweiller <ngeiswei@gmail.com>
 *         Linas Vepstas <linasvepstas@gmail.com>
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

#include <signal.h>
#include <unistd.h>

#include <boost/algorithm/string/trim.hpp>
#include <boost/range/algorithm/transform.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>

#include <opencog/util/log_prog_name.h>
#include <opencog/util/numeric.h>
#include <opencog/util/oc_omp.h>

#include <opencog/comboreduct/table/table.h>
#include <opencog/comboreduct/table/table_io.h>

#include "moses_exec.h"
#include "moses_exec_def.h"
#include "demo-problems.h"
#include "problem.h"

#include "../moses/moses_main.h"
#include "../moses/partial.h"
#include "../optimization/hill-climbing.h"
#include "../optimization/optimization.h"
#include "../example-progs/scoring_iterators.h"



namespace opencog { namespace moses {

using boost::format;
using boost::trim;
using boost::str;
using namespace boost::program_options;
using boost::lexical_cast;
using namespace std;
using namespace reduct;

static const unsigned int max_filename_size = 255;

// default number of samples to describe a problem
static const unsigned int default_nsamples = 20;


// problems
static const string it="it"; // regression based on input table
                             // maximize accuracy.

static const string recall="recall"; // regression based on input table,
                                  // maximize recall, while holding
                                  // precision const.

static const string prerec="prerec"; // regression based on input table,
                                  // maximize precision, while holding
                                  // recall const.

static const string bep="bep";    // regression based on input table,
                                  // maximize break-even point

static const string f_one="f_one"; // regression based on input table,
                                  // maximize f_1 score

static const string pre="pre";    // regression based on input table by
                                  // maximizing precision (or negative
                                  // predictive value), holding activation
                                  // const.

static const string pre_conj="pre_conj";    // simplified version of
                                            // pre, but tries to
                                            // maximize number of
                                            // conjunctions

static const string ip="ip"; // find interesting patterns

/* ANN problems */
static const string ann_it="ann-it"; // regression based on input table using ann
static const string ann_cp="ann-cp"; // regression based on combo program using ann

/* Demo problems */
static const string pa="pa"; // even parity
static const string cp="cp"; // regression based on combo program to fit
static const string ann_xor="ann-xor"; // binary-xor problem using ann
static const string ann_pole1="ann-pole1"; // pole balancing problem using ann
static const string ann_pole2="ann-pole2"; // double pole balancing problem ann

// diversity distance types
static const string p_norm = "p_norm";
static const string tanimoto = "tanimoto";
static const string angular = "angular";
        
// diversity dst2dp types
static const string auto_str = "auto";        
static const string inverse_str = "inverse";
static const string complement_str = "complement";
static const string power_str = "power";

// focus types (which data points feature selection within moses
// should focus on)
static const string focus_all = "all"; // all data points are considered

static const string focus_active = "active"; // only active data points are
                                             // considered
static const string focus_incorrect = "incorrect"; // only incorrect answers
                                                   // are considered
static const string focus_ai = "ai"; // only active data points that
                                     // are incorrectly classified are considered

// seed type (how to use the features of the exemplar to seed feature
// selection)

// empty initial feature set, however the features of the exemplar are
// simply removed from the dataset before feature selection
// occurs. This is to prevent that new selected features are ones from
// the exemplar.
static const string seed_none = "none";

// empty initial feature set, the features of the exemplar are not
// removed from the dataset but the number of features of the exemplar
// is added to the number of features to select. That is (for that
// particular expansion):
//
// fs-target-size += number of features in exemplar
static const string seed_add = "add";

// the features of the exemplar are used as initial guess for feature
// selection, also the number of features to select is also added to
// the number of features of the exemplar, as for add
static const string seed_init = "init";

// the "exemplar feature" us used as initial guess. The exemplar
// feature is the output of the exemplar. The number of features to
// select is incremented by 1 (to account for the exemplar
// feature). that is the number of feature to select is fs-target-size + 1
static const string seed_xmplr = "xmplr";

void log_output_error_exit(string err_msg) {
    logger().info() << "Error: " << err_msg;
    cerr << "Error: " << err_msg << endl;
    exit(1);    
}

/**
 * Display error message about unspecified combo tree and exit
 */
void unspecified_combo_exit()
{
    log_output_error_exit("you must specify which combo tree to learn (option -y).");
}

/**
 * Display error message about unsupported type and exit
 */
void unsupported_type_exit(const type_tree& tt)
{
    stringstream ss;
    ss << "type " << tt << " currently not supported.";
    log_output_error_exit(ss.str());
}
void unsupported_type_exit(type_node type)
{
    unsupported_type_exit(type_tree(type));
}

/**
 * Display error message about ill formed combo tree and exit
 */
void illformed_exit(const combo_tree& tr)
{
    stringstream ss;
    ss << "the combo tree " << tr << " is not well formed.";
    log_output_error_exit(ss.str());
}

/**
 * Display error message about unsupported problem and exit
 */
void unsupported_problem_exit(const string& problem)
{
    stringstream ss;
    ss << "problem type \"" << problem << "\" is currently unsupported.";
    log_output_error_exit(ss.str());
}

/**
 * Display error message about missing input data file and exit
 */
void no_input_datafile_exit()
{
    stringstream ss;
    ss << "no input data file has been specified (option -"
       << input_data_file_opt.second << ")";
    log_output_error_exit(ss.str());
}

/**
  * Display error that not all data files have same arity and exit
  */
void not_all_same_arity_exit(const string& input_data_file1, arity_t arity1,
                             const string& input_data_file2, arity_t arity2)
{
    stringstream ss;
    ss << "File " << input_data_file1 << " has arity " << arity1
       << " while file " << input_data_file2 << "has_arity " << arity2;
    log_output_error_exit(ss.str());
}

/**
 * Display error message about not recognized combo operator and exist
 */
void not_recognized_combo_operator(const string& ops_str)
{
    stringstream ss;
    ss << ops_str << " is not recognized as combo operator.";
    log_output_error_exit(ss.str());
}

/**
 * Display error message about not recognized diversity distance and exist
 */
void not_recognized_dst(const string& diversity_dst)
{
    stringstream ss;
    ss << diversity_dst << " is not recognized. Valid distances are "
       << p_norm << ", " << tanimoto << " and " << angular;
    log_output_error_exit(ss.str());
}

/**
 * Display error message about not recognized diversity distance to
 * penalty function and exist
 */
void not_recognized_dst2dp(const string& diversity_dst2dp)
{
    stringstream ss;
    vector<string> valid_dsts = {auto_str, inverse_str, complement_str, power_str};
    ostreamContainer(ss << diversity_dst2dp
                     << " is not recognized. Valid distances to penalty are ",
                     valid_dsts, ", ");
    log_output_error_exit(ss.str());
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
    type_node output_type = get_type_node(get_signature_output(tt));

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
    if (noise >= 0.0)
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
    static set<string> dbp
        = {it, pre, pre_conj, recall, prerec, bep, f_one, ann_it, ip};
    return dbp.find(problem) != dbp.end();
}

//* return true iff the problem is based on a combo tree
bool combo_based_problem(const string& problem)
{
    return problem == cp || problem == ann_cp;
}

// Infer the arity of the problem
combo::arity_t infer_arity(const string& problem,
                           unsigned int problem_size,
                           const string& combo_str)
{
    if (datafile_based_problem(problem)) {
        return -1;
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
    else if (problem == pa)
    {
        return problem_size;
    }
    else
    {
        problem_base* probm = find_problem(problem);
        if (probm) return probm->arity(problem_size);

        unsupported_problem_exit(problem);
        return -1;
    }
}



int moses_exec(int argc, char** argv)
{
    // for(int i = 0; i < argc; ++i)
    //     cout << "arg = " << argv[i] << endl;

    // program options, see options_description below for their meaning
    vector<string> jobs_str;
    unsigned min_pool;
    bool enable_mpi = false;

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
    time_t max_time;
    long result_count;
    bool output_score;
    bool output_penalty;
    bool output_bscore;
    bool output_only_best;
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
    vector<string> ignore_features_str;
    string opt_algo; //optimization algorithm
    vector<string> exemplars_str;
    int reduct_candidate_effort;
    int reduct_knob_building_effort;
    bool weighted_accuracy;

    // metapop_param
    int max_candidates;
    bool reduce_all;
    unsigned revisit;
    score_t complexity_temperature = 5.0f;
    score_t complexity_ratio = 3.5f;
    double cap_coef;
    unsigned cache_size;
    bool linear_regression;
    float perm_ratio;
    
    // diversity parameters
    bool include_dominated;
    score_t diversity_pressure;
    score_t diversity_exponent;
    bool diversity_normalize;
    string diversity_dst;
    score_t diversity_p_norm;
    string diversity_dst2dp;

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
    score_t hardness;       // hardness of the activation range
                            // constraint for problems pre, recall, prerec
    // hc_param
    bool hc_widen_search;
    bool hc_single_step;
    bool hc_crossover;
    unsigned hc_crossover_pop_size;
    bool hc_allow_resize_deme;
    unsigned hc_max_nn;
    double   hc_frac_of_nn;

    // classifier paramters
    bool use_well_enough = false;

    // pre params
    bool pre_worst_norm;
    bool gen_best_tree;

    // it params
    bool it_abs_err;

    // EXPERIMENTAL
    // feature selection happens before each representation building
    /// Enable feature selection while selecting exemplar
    bool enable_feature_selection;
    string fs_focus;
    string fs_seed;
    feature_selector_parameters festor_params;
    feature_selection_parameters& fs_params = festor_params.fs_params;

    // Declare the supported options.
    // XXX TODO: make this print correctly, instead of using brackets.
    options_description desc("Allowed options");
    desc.add_options()

        // General options
        
        ("help,h", "Produce help message.\n")

        ("version", "Display the version of moses.\n")

        (opt_desc_str(jobs_opt).c_str(),
         value<vector<string>>(&jobs_str),
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

        ("min-pool", value<unsigned>(&min_pool)->default_value(50),
         "Minimum number of elements to process in the pool to enable "
         " multi-threading.\n")

        (opt_desc_str(exemplars_str_opt).c_str(),
         value<vector<string>>(&exemplars_str),
         "Start the search with a given exemplar, can be used several times.\n")

        // Problem-type options
        
        (opt_desc_str(problem_opt).c_str(),
         value<string>(&problem)->default_value(it),
         str(format("Problem to solve, supported problems are:\n\n"
                    "%s, regression based on input table\n\n"
                    "%s, regression based on input table, maximizing precision, while holding activation fixed\n\n"
                    "%s, regression based on input table, maximizing precision, while holding recall fixed\n\n"
                    "%s, regression based on input table, maximizing recall while holding precision fixed\n\n"
                    "%s, regression based on input table, maximizing break-even point (BEP) between precision and recall\n\n"
                    "%s, regression based on input table, maximizing the F_1 score (harmonic mean of precision and recall)\n\n"
                    "%s, search interesting patterns, where interestingness"
                    " is defined in terms of several features such as maximizing"
                    " the Kullback-Leibler"
                    " divergence between the distribution of the outputs and"
                    " that same distribution in the context of the pattern"
                    " being true."
                    " Or the difference of skewnesses between the 2 distributions"
                    " and other things being experimented.\n\n"
                    "%s, regression based on input table using ann\n\n"
                    "%s, demo, regression based on combo program\n\n"
                    "%s, demo, even parity problem\n\n"
                    "dj, demo, disjunction problem\n\n"
                    "mux, demo, multiplex problem\n\n"
                    "maj, demo, majority problem\n\n"
                    "sr, demo, regression of f_n(x) = sum_{k=1,n} x^k\n")
             % it % pre % prerec % recall % bep % f_one % ip % ann_it % cp % pa).c_str())

        // Input specification options

        (opt_desc_str(input_data_file_opt).c_str(),
         value<vector<string>>(&input_data_files),
         "Input table file in DSV format (with comma, whitespace "
         "and tabulation as seperator). Colums correspond to features "
         "and rows to observations. Can be used several times, in such "
         "a case the behavioral score of the whole problem is the "
         "concatenation of the behavioral scores of the sub-problems "
         "associated with the files. Each file must have the same number "
         "of features in the same order.\n")

        (opt_desc_str(target_feature_opt).c_str(),
         value<string>(&target_feature),
         "Label of the target feature to fit. If none is given the first one is used.\n")

        (opt_desc_str(ignore_feature_str_opt).c_str(),
         value<vector<string>>(&ignore_features_str),
         "Ignore feature from the datasets. Can be used several times "
         "to ignore several features.\n")

        (opt_desc_str(nsamples_opt).c_str(),
         value<int>(&nsamples)->default_value(-1),
         "Number of samples to describe the problem. "
         "If nsample is negative, null or larger than the maximum "
         "number of samples allowed it is ignored. If the default "
         "problem size is larger than the value provided with that "
         "option then the dataset is subsampled randomly to reach the "
         "target size.\n")

        // Algorithm control options

        (opt_desc_str(hc_crossover_opt).c_str(),
         value<bool>(&hc_crossover)->default_value(true),
         str(format("Hillclimbing parameter (%s). If false, then only "
                    "the local neighborhood of the current center "
                    "instance is explored. That is, the highest-scoring "
                    "instance is chosen as the new center "
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

        ("hc-crossover-pop-size",
         value<unsigned>(&hc_crossover_pop_size)->default_value(120),
         "Number of new candidates created by crossover during each iteration "
         "of hillclimbing. It also allows to control when crossover occurs over "
         "exhaustive search. Specifically if the number of candidate to explore "
         "by exhaustive search is more than 10/3 * crossover_pop_size, then "
         "crossover kicks in.\n")

        ("hc-allow-resize-deme",
         value<bool>(&hc_allow_resize_deme)->default_value(true),
         str(format("Hillclimbing parameter (%s). If true then the deme "
                    "is allowed to resized to fit in memory. Not that as it "
                    "uses the RAM of the machine therefore it possibly introduces "
                    "an indeterminism between instances run on machines with "
                    "different RAM.\n") % hc).c_str())

        (opt_desc_str(opt_algo_opt).c_str(),
         value<string>(&opt_algo)->default_value(hc),
         str(format("Optimization algorithm, supported algorithms are"
                    " univariate (%s), simulation annealing (%s),"
                    " hillclimbing (%s).\n")
             % un % sa % hc).c_str())

        (opt_desc_str(max_score_opt).c_str(),
         value<score_t>(&max_score)->default_value(very_best_score),
         "The max score to reach, once reached MOSES halts. If the largest "
         "floating point number is used and MOSES is able to calculate the "
         "max score that can be reached for that particular problem, "
         "it will overwrite it. "
         "Otherwise, for any other value, the user's defined max-score will "
         "be used.\n")

        (opt_desc_str(max_evals_opt).c_str(),
         value<unsigned long>(&max_evals)->default_value(10000),
         "Maximum number of fitness function evaluations.\n")

        ("max-time",
         value<time_t>(&max_time)->default_value(INT_MAX),
         "Longest allowed runtime, in seconds.\n")

        (opt_desc_str(cache_size_opt).c_str(),
         value<unsigned>(&cache_size)->default_value(100000),
         "Cache size. Memoize, that is, cache evaluation results, "
         "so that identical candidates are not re-evaluated.\n")
         // adaptive_cache has been temporarly disabled because it is
         // not thread safe, so the following comment doesn't apply
         // " The cache size is determined by this option "
         // "adjusted to fit in the RAM.\n")

        (opt_desc_str(ignore_ops_str_opt).c_str(),
         value<vector<string>>(&ignore_ops_str),
         str(format("Ignore the following operator in the program solution.  "
                    "This option may be used several times.  Currently, only div, "
                    "sin, exp, log can be ignored. "
                    "This option has the priority over --%s. "
                    "That is, if an operator is both be included and ignored, "
                    "then it is ignored.  This option does not work with ANN.\n")
             % include_only_ops_str_opt.first).c_str())

        ("linear-regression",
         value<bool>(&linear_regression)->default_value(false),
         "When attempting to fit continous-valued features, restrict "
         "searches to linear expressions only; that is, do not use "
         "polynomials in the fit.  Specifying this option also "
         "automatically disables the use of div, sin, exp and log.\n")

        ("logical-perm-ratio",
         value<float>(&perm_ratio)->default_value(0.0),
         "When decorating boolean exemplars with knobs, this option "
         "controls how many pairs of literals of the form op(L1 L2) are "
         "created.  That is, such pairs are used to decorate the exemplar "
         "tree. By default, N such pairs are created, where N is the "
         "arity of the problem. Valid values for this option are in the "
         "range of -1.0 to +1.0.  For negative values, the number of pairs "
         "created are (1.0+r)*N where r is the value given in this option.  "
         "For positive values, the number of pairs created is "
         "(N + r*N*(N-2)). A larger number of pairs can lead to faster "
         "solutions but can also result in over-training.\n")

        (opt_desc_str(rand_seed_opt).c_str(),
         value<unsigned long>(&rand_seed)->default_value(1),
         "Random seed.\n")

        (opt_desc_str(complexity_temperature_opt).c_str(),
         value<score_t>(&complexity_temperature)->default_value(6.0),
         "Set the \"temperature\" (scritly positive floating number) "
         "of the Boltzmann-like distribution "
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
         "determined by earlier runs).\n")

        ("cap-coef", value<double>(&cap_coef)->default_value(50.0),
         "Set the leading coefficient of the formula defining the "
         "metapop size cap = cap_coef*(x+250)*(1+2*exp(-x/500)), "
         "where x is the number of generations so far. The default usually works "
         "well but if you run out of memory you may decrease that value.\n")
        
        // Large problem parameters

        ("hc-max-nn-evals",
         value<unsigned>(&hc_max_nn)->default_value(20000),
         str(format("Hillclimbing parameter (%s).  When exploring the "
         "nearest neighborhood of an instance, this number specifies "
         "the maximum number of nearest neighbors to explore.  An "
         "exhaustive search of the nearest neighborhood is performed "
         "when the number of nearest neighbors is less than this value.  "
         "Problems with a large number of features (100 and above) often "
         "evolve exemplars with a complexity of 100 or more, which in turn "
         "may have instances with hundreds of thousands of nearest neighbors.  "
         "Exploring one nearest neighbor requires one evaluation of the "
         "scoring function, and so an exhaustive search can be prohibitive.  "
         "A partial search can often work quite well, especially when "
         "cross-over is enabled.\n") % hc).c_str())

        ("hc-fraction-of-nn",
         value<double>(&hc_frac_of_nn)->default_value(2.0),
         str(format("Hillclimbing parameter (%s).  When exploring the "
         "nearest neighborhood of an instance, this number specifies "
         "the fraction of nearest neighborhood to explore.  As currently "
         "implemented, only an estimate of the nearest-neighborhood size "
         "is used, not the true size.  However, this estimate is accurate "
         "to within a factor of 2.  Thus, to obtain an exhaustive search "
         "of the entire neighborhood, set this to 2.0 or larger.  "
         "Problems with a large number of features (100 and above) often "
         "evolve exemplars with a complexity of 100 or more, which in turn "
         "may have instances with hundreds of thousands of nearest neighbors.  "
         "Exploring one nearest neighbor requires one evaluation of the "
         "scoring function, and so an exhaustive search can be prohibitive.  "
         "A partial search can often work quite well, especially when "
         "cross-over is enabled.\n") % hc).c_str())

        // Algorithm tuning options
        
        (opt_desc_str(reduct_knob_building_effort_opt).c_str(),
         value<int>(&reduct_knob_building_effort)->default_value(2),
         "Effort allocated for reduction during knob building, 0-3, "
         "0 means minimum effort, 3 means maximum effort. The bigger "
         "the effort the lower the dimension of the deme.\n")

        (opt_desc_str(max_dist_opt).c_str(),
         value<size_t>(&max_dist)->default_value(4),
         "The maximum radius of the neighborhood around the "
         "exemplar to explore.\n")

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

        (opt_desc_str(max_gens_opt).c_str(),
         value<int>(&max_gens)->default_value(-1),
         "Maximum number of demes to generate and optimize, "
         "negative means no generation limit.\n")

        (opt_desc_str(include_dominated_opt).c_str(),
         value<bool>(&include_dominated)->default_value(true),
         "Include dominated candidates (according behavioral score) "
         "when merging candidates in the metapopulation. Disabling "
         "this may lead to poorer performance.\n")

        (opt_desc_str(hc_single_step_opt).c_str(),
         value<bool>(&hc_single_step)->default_value(false),
         str(format("Hillclimbing parameter (%s). If false, then the normal "
                    "hillclimbing algorithm is used.  If true, then only one "
                    "step is taken towards the hilltop, and the results are "
                    "promptly folded back into the metapopulation. If this "
                    "flag is set, then consider using the widen-search flag "
                    "as well, so as to make forward progress.\n") % hc).c_str())

        (opt_desc_str(include_only_ops_str_opt).c_str(),
         value<vector<string>>(&include_only_ops_str),
         "Include this operator, but exclude others, in the solution.  "
         "This option may be used several times to specify multiple "
         "operators.  Currently, only these operators are "
         "supported: plus, times, div, sin, exp, log. "
         "This option does not work with ANN.\n")

        (opt_desc_str(pop_size_ratio_opt).c_str(),
         value<double>(&pop_size_ratio)->default_value(20),
         "The higher the more effort is spent on a deme.\n")

        (opt_desc_str(noise_opt).c_str(),
         value<float>(&noise)->default_value(-1),
         "Alternative way to set the ratio of raw score to complexity.  "
         "Setting this option over-rides the complexity ratio, above.  "
         "Assumes that the data is noisy.   The noisier the data, the "
         "stronger the model complexity penalty.  If the target feature "
         "is discrete, the setting should correspond to the fraction of "
         "the input data that might be wrong (i.e. the probability p "
         "that an output datum (row) is wrong).   In this case, only "
         "values 0 <= p < 0.5 are meaningful (i.e. less than half the "
         "data can be wrong). Suggested values are in the range 0.01 to "
         "0.001.  If the target feature is continuous, the value specified "
         "should correspond to the standard deviation of the (Gaussian) "
         "noise centered around each candidate's output. A negative "
         "value cedes this setting to complexity-ratio flag, above.\n")

        (opt_desc_str(hc_widen_search_opt).c_str(),
         value<bool>(&hc_widen_search)->default_value(false),
         str(format("Hillclimbing parameter (%s). If false, then deme search "
                    "terminates when a local hilltop is found. If true, "
                    "then the search radius is progressively widened, "
                    "until another termination condition is met.\n") % hc).c_str())

        ("well-enough",
         value<bool>(&use_well_enough)->default_value(false),
         "If 1, use the \"leave well-enough alone\" algorithm for "
         "classification problems. This algorithm, after finding a "
         "clause that has perfect accuracy, will stop mutating that "
         "clause any further. In principle, this should speed "
         "convergence.  In practice, not so much; it can hurt "
         "performance.\n")

        ("revisit", value<unsigned>(&revisit)->default_value(0),
         "Number of times the same exemplar can be revisited. "
         "This option is only worthwhile when there "
         "is a great deal of stochasticity in the search so that exploring "
         "a deme multiple times will yield substantially different results. "
         "This might be the case is feature selection is used for instance.\n")

        // Output control options
        
        (opt_desc_str(result_count_opt).c_str(),
         value<long>(&result_count)->default_value(10),
         "The number of results to return, ordered according to "
         "a linear combination of score and complexity. If negative, "
         "then return all results.\n")

        (opt_desc_str(output_score_opt).c_str(),
         value<bool>(&output_score)->default_value(true),
         "If 1, output the score before each candidate (at the left of the complexity).\n")

        (opt_desc_str(output_penalty_opt).c_str(),
         value<bool>(&output_penalty)->default_value(false),
         "If 1, output the penalized score and it's compenents (below each candidate).\n")

        (opt_desc_str(output_bscore_opt).c_str(),
         value<bool>(&output_bscore)->default_value(false),
         "If 1, output the bscore (below each candidate).\n")

        (opt_desc_str(output_only_best_opt).c_str(),
         value<bool>(&output_only_best)->default_value(false),
         "If 1, print only the best candidates.\n")

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

        // Demo options
        
        (opt_desc_str(combo_str_opt).c_str(),
         value<string>(&combo_str),
         str(format("Combo program to learn, used when the problem"
                    " %s is selected (option -%s).\n")
             % cp % problem_opt.second).c_str())

        (opt_desc_str(problem_size_opt).c_str(),
         value<unsigned int>(&problem_size)->default_value(5),
         str(format("For even parity (%s), disjunction (dj) and majority (maj) "
                    "the problem size corresponds directly to the arity. "
                    "For multiplex (mux) the arity is arg+2^arg. "
                    "For regression of f(x)_o = sum_{i={1,o}} x^i (sr) "
                    "the problem size corresponds to the order o.\n")
             % pa).c_str())

        // The remaining options (TODO organize that)
        
        (opt_desc_str(min_rand_input_opt).c_str(),
         value<float>(&min_rand_input)->default_value(0.0),
         "Minimum value of a sampled coninuous input.  The cp, ip, pre, "
         "recall, prerec, bep and f_one "
         "problems all require a range of values to be sampled in "
         "order to measure the fitness of a proposed solution. This "
         "option sets the low end of the sampled range. In the case of "
         "fitness function pre, the range corresponds to the activation "
         "of the precision.\n")

        (opt_desc_str(max_rand_input_opt).c_str(),
         value<float>(&max_rand_input)->default_value(1.0),
         "Maximum value of a sampled coninuous input.  The cp, ip, pre, "
         "recall, prerec, bep and f_one "
         "problems all require a range of values to be sampled in "
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

        (opt_desc_str(max_candidates_opt).c_str(),
         value<int>(&max_candidates)->default_value(-1),
         "Maximum number of considered candidates to be added to the "
         "metapopulation after optimizing deme.\n")

#ifdef HAVE_MPI
        ("mpi",
         value<bool>(&enable_mpi)->default_value(false),
         "Enable MPI-based distributed processing.\n")
#endif

        (opt_desc_str(weighted_accuracy_opt).c_str(),
         value<bool>(&weighted_accuracy)->default_value(false),
         "This option is useful in case of unbalanced data as it "
         "weights the score so that each class weights equally "
         "regardless of their proportion in terms of sample size.\n")

        ("diversity-pressure",
         value<score_t>(&diversity_pressure)->default_value(0.0),
         "Set a diversity pressure on the metapopulation. "
         "Programs behaving similarily to others are more penalized. "
         "That value sets the importance of that penalty (from 0 to +inf).\n")

        ("diversity-exponent",
         value<score_t>(&diversity_exponent)->default_value(-1.0),
         "Set the exponent of the generalized mean (or sum, if "
         "--diversity-normalize is set to 0) aggregating "
         "the penalties between a candidate and the set of all candidates better "
         "than itself (taking into account diversity). If the value tends "
         "towards 0 it tends to the geometric mean, towards +inf it tends "
         "to the max function. If negative or null is it the max function.\n")

        ("diversity-normalize",
         value<bool>(&diversity_normalize)->default_value(true),
         "If set to 1 then the aggregating function is a generalized mean. "
         "Otherwize it is a generalized sum (generalize mean * number of "
         "elements). If --diversity-exponent is set to negatively then "
         "this doesn't have any impact as the aggregating function is "
         "the max anyway.\n")

        ("diversity-dst",
         value<string>(&diversity_dst)->default_value(p_norm),
         str(format("Set the distance between behavioral scores, "
                    "then used to determin the diversity penalty."
                    "3 distances are available: %s, %s and %s.\n")
             % p_norm % tanimoto % angular).c_str())

        ("diversity-p-norm",
         value<score_t>(&diversity_p_norm)->default_value(2.0),
         "Set the parameter of the p_norm distance. A value of 1.0"
         "correspond to the Manhatan distance. A value of 2.0 corresponds to "
         "the Euclidean distance. A value of 0.0 or less correspond to the "
         "max component-wise. Any other value corresponds to the general case.\n")

        ("diversity-dst2dp",
         value<string>(&diversity_dst2dp)->default_value(auto_str),
         str(format("Set the type of function to convert distance into penalty. "
                    "4 options are available: %1%, %2%, %3% and %4%. "
                    "When %1% is selected the function is selected depending "
                    "on the distance, if the distance is %5%, "
                    "then %2% is selected, otherwise %3% is selected.\n")
             % auto_str % inverse_str % complement_str
             % power_str % p_norm).c_str())

        (opt_desc_str(discretize_threshold_opt).c_str(),
         value<vector<contin_t>>(&discretize_thresholds),
         "If the domain is continuous, discretize the target feature. "
         "A unique used of that option produces 2 classes, x < thresold "
         "and x >= threshold. The option can be used several times (n-1) "
         "to produce n classes and the thresholds are automatically sorted.\n")

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
         value<score_t>(&hardness)->default_value(0.0),
         "If problems pre, prerec, recall, f_one or bep are specified, "
         "this option is used to set the 'hardness' of the constraint, "
         "with larger values corresponding to a harder constraint "
         "(i.e. punishing the score more strongly if the contraint "
         "is not met.)  For the 'pre' problem, if alpha is negative, "
         "then its absolute value is used for the hardness, and "
         "the negative predictive value is maximized (instead of "
         "the precision).\n")

        ("pre-worst-norm",
         value<bool>(&pre_worst_norm)->default_value(false),
         "Normalize the precision w.r.t. its worst decile [EXPERIMENTAL].\n")

        ("it-abs-err",
         value<bool>(&it_abs_err)->default_value(false),
         "Use absolute error instead of squared error [EXPERIMENTAL, the occam's razor hasn't been calibrated for that fitness function yet].\n")

        ("gen-best-tree",
         value<bool>(&gen_best_tree)->default_value(false),
         "Attempts to generate the best candidate (possibly huge and overfit) head-on. Only works combined with -Hpre for now.\n")

        // ======= Feature-selection params =======
        ("enable-fs",
         value<bool>(&enable_feature_selection)->default_value(false),
         "Enable integrated feature selection.  Feature selection is "
         "performed immediately before knob building (representation "
         "building), when creating a new deme.  Limiting the number "
         "of features can sharply improve the run-time and memory "
         "usage of large problems.\n")

        ("fs-target-size",
         value<unsigned>(&fs_params.target_size)->default_value(20),
         "Feature count.  This option "
         "specifies the number of features to be selected out of "
         "the dataset.  A value of 0 disables feature selection.\n")

        ("fs-exp-distrib",
         value<bool>(&fs_params.exp_distrib)->default_value(false),
         "Use a smoth exponential distribution, instead of hard "
         "cuttoff, when selecting the highest-scoring features.  "
         "Without this option, the highest-scoring count=N features "
         "will be selected. That is, the distribution will be a hard "
         "cutoff or cliff: after ranking all features by score, the "
         "k'th highest-ranked feature will be selected with probability "
         "1.0 if k<N  and with probability 0.0 if k>N.  With this option "
         "enabled, a total of count=N features will still be selected, "
         "and most of these will be the highest scoring ones, but a few "
         "lower-ranked features will also be included.  Specifically, "
         "the probability of choosing the k'th ranked feature will be "
         "exp(-tk) with t choosen so that, on average, N features are "
         "selected.  The initial random seed affects the generated list. "
         "Currently, this option only applies to the -asimple algo, and "
         "is ignored by the others (this needs to be fixed.)\n")


        ("fs-focus",
         value<string>(&fs_focus)->default_value(focus_incorrect),
         str(boost::format("Focus of feature selection (which data points "
                           "feature will focus on):\n\n"
                           "%s, all data points are considered\n\n"
                           "%s, only active data points are considered\n\n"
                           "%s, only incorrect answers are considered\n\n"
                           "%s, only active data points that are incorrectly "
                           "answered are considered.\n")
             % focus_all % focus_active % focus_incorrect % focus_ai).c_str())

        ("fs-seed",
         value<string>(&fs_seed)->default_value(seed_add),
         str(boost::format("Seed type (how to use the features of the "
                           "exemplar to seed feature selection):\n\n"

                           "%s, empty initial feature set.  The "
                           "features used in the exemplar are removed "
                           "from the dataset before feature selection occurs. "
                           "This prevents newly selected features from "
                           "being those already in the exemplar.\n\n"

                           "%s, empty initial feature set. The number of "
                           "features currently in the exemplar are added"
                           "to the number of features to be selected. "
                           "This guarentees that at least fs_target_size "
                           "features are not from the exemplar itself.\n\n"

                           "%s, the features of the exemplar are used as "
                           "an initial guess for feature selection. "
                           "The number of "
                           "features currently in the exemplar are added"
                           "to the number of features to be selected.\n\n"

                           "%s, the \"exemplar feature\" is used as initial "
                           "guess. The exemplar feature is the output of the "
                           "exemplar. The number of features to select is "
                           "incremented by 1 (to account for the exemplar "
                           "feature). That is, the number of feature to select "
                           "is fs_target_size + 1\n")
             % seed_none % seed_add % seed_init % seed_xmplr).c_str())

        ("fs-prune-exemplar",
         value<bool>(&festor_params.prune_xmplr)->default_value(0),
         "Remove from the exemplar the literals of non-selected features.\n")

        ("fs-subsampling-pbty",
         value<float>(&festor_params.subsampling_pbty)->default_value(0),
         "Probability of discarding an observation before carrying feature "
         "selection. 0 means no observation is discard, 1 means all are discard. "
         "This is to force to introduce some randomness in "
         "feature selection, as not all feature selection algorithms "
         "have some.\n")

        ("fs-demes",
         value<unsigned>(&festor_params.n_demes)->default_value(1),
         "Number of feature sets to select out of feature selection and the "
         "number of demes to accordingly spawn.\n")

        ("fs-algo",
         value<string>(&fs_params.algorithm)->default_value(simple),
         string("Feature selection algorithm. Supported algorithms are:\n")
         .append(simple).append(" for a simple, fast max-mutual-information algo.\n")
         .append(inc).append(" for incremental max-relevency, min-redundancy.\n")
         .append(smd).append(" for stochastic maximal dependency,\n")
         .append(moses::hc).append(" for moses-hillclimbing.\n").c_str())

        ("fs-scorer",
         value<string>(&fs_params.scorer)->default_value(mi),
         str(boost::format("Feature selection fitness function (scorer).\n"
                           " Supported scorers are:\n"
                           "%s, for mutual information\n"
                           "%s, for precision (see moses -h for more info)\n")
             % mi % pre).c_str())

        ("fs-threshold",
         value<double>(&fs_params.threshold)->default_value(0),
         "Improvement threshold. Floating point number. "
         "Specifies the threshold above which the mutual information "
         "of a feature is considered to be significantly correlated "
            "to the target.  A value of zero means that all features "
         "will be selected. \n"
         "For the -ainc algo only, the -C flag over-rides this setting.\n")

        // ======= Feature-selection diveristy pressure =======
        ("fs-diversity-pressure",
         value<float>(&festor_params.diversity_pressure)->default_value(0),
         "Multiplicative coefficient of the diversity penalty "
         "(itself being in [0,1]).\n")

        ("fs-diversity-cap",
         value<size_t>(&festor_params.diversity_cap)->default_value(100),
         "Place a cap on the maximum number of feature set to consider. "
         "If zero, no cap is used (Warning: could be very slow). "
         "Use this to speed up diversity computation on feature sets.\n")

        ("fs-diversity-interaction",
         value<int>(&festor_params.diversity_interaction)->default_value(-1),
         "Maximum number of interactions to be considered when computing "
         "the mutual information between feature sets. "
         "This is used in case the number of selected features tends to "
         "be high compared to the number of datapoints to decrease inacuracy "
         "of the mutual information.\n")

        // ======= Feature-selection incremental algo params =======
        ("fs-inc-redundant-intensity",
         value<double>(&fs_params.inc_red_intensity)->default_value(-1.0),
         "Incremental Selection parameter. Floating-point value must "
         "lie between 0.0 and 1.0.  A value of 0.0 or less means that no "
         "redundant features will discarded, while 1.0 will cause a "
         "maximal number will be discarded.\n")

        ("fs-inc-target-size-epsilon",
         value<double>(&fs_params.inc_target_size_epsilon)->default_value(1.0e-6),
         "Incremental Selection parameter. Tolerance applied when "
         "selecting for a fixed number of features (option -C).\n")

        ("fs-inc-interaction-terms",
         value<unsigned>(&fs_params.inc_interaction_terms)->default_value(1),
         "Incremental Selection parameter. Maximum number of "
         "interaction terms considered during incremental feature "
         "selection. Higher values make the feature selection more "
         "accurate but is combinatorially more computationally expensive.\n")

        // ======= Feature-selection pre scorer only params =======
        ("fs-pre-penalty",
         value<float>(&fs_params.pre_penalty)->default_value(1.0f),
         "Activation penalty (see moses --help or man moses for more info).\n")

        ("fs-pre-min-activation",
         value<float>(&fs_params.pre_min_activation)->default_value(0.5f),
         "Minimum activation (see moses --help or man moses for more info).\n")

        ("fs-pre-max-activation",
         value<float>(&fs_params.pre_max_activation)->default_value(1.0f),
         "Maximum activation (see moses --help or man moses for more info).\n")

        ("fs-pre-positive",
         value<bool>(&fs_params.pre_positive)->default_value(true),
         "If 1, then precision, otherwise negative predictive value "
         "(see moses --help or man moses for more info).\n")

        // ======= Feature-selection hill-climbing only params =======
        ("fs-hc-max-score",
         value<double>(&fs_params.hc_max_score)->default_value(1),
         "Hillclimbing parameter.  The max score to reach, once "
         "reached feature selection halts.\n")

        // no need of that for now
        // (opt_desc_str(hc_initial_feature_opt).c_str(),
        //  value<vector<string> >(&fs_params.hc_initial_features),
        //  "Hillclimbing parameter.  Initial feature to search from.  "
        //  "This option can be used as many times as there are features, "
        //  "to have them included in the initial feature set. If the "
        //  "initial feature set is close to the one that maximizes the "
        //  "quality measure, the selection speed can be greatly increased.\n")

        ("fs-hc-max-evals",
         value<unsigned>(&fs_params.hc_max_evals)->default_value(10000),
         "Hillclimbing parameter.  Maximum number of fitness function "
         "evaluations.\n")

        ("fs-hc-fraction-of-remaining",
         value<double>(&fs_params.hc_fraction_of_remaining)->default_value(0.5),
         "Hillclimbing parameter.  Determine the fraction of the "
         "remaining number of eval to use for the current iteration.\n")

        ("fs-hc-crossover",
         value<bool>(&fs_params.hc_crossover)->default_value(false),
         "Hillclimber crossover (see --hc-crossover option)\n")

        ("fs-hc-crossover-pop-size",
         value<unsigned>(&fs_params.hc_crossover_pop_size)->default_value(false),
         "Hillclimber crossover pop size (see --hc-crossover option)\n")

        ("fs-hc-widen-search",
         value<bool>(&fs_params.hc_widen_search)->default_value(true),
         "Hillclimber widen_search (see --widen-search)\n")

        // ======= Feature-selection MI scorer params =======
        ("fs-mi-penalty",
         value<double>(&fs_params.mi_confi)->default_value(100.0),
         "Mutual-information scorer parameter.  Intensity of the confidence "
         "penalty, in the range (-Inf, +Inf).  100 means no confidence "
         "penalty. This parameter influences how much importance is "
         "attributed to the confidence of the quality measure. The "
         "fewer samples in the data set, the more features the "
         "less confidence in the feature set quality measure.\n")

        // ======= Feature-selection SMD params =======
        ("fs-smd-top-size",
         value<unsigned>(&fs_params.smd_top_size)->default_value(10),
         "Stochastic max dependency parameter. Number of feature subset "
         "candidates to consider building the next superset.\n")

        // ========== THE END of the options; note semicolon ===========
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
        cout << "moses " << version_string << std::endl;
#ifdef HAVE_MPI
        cout << "\tMPI support enabled." << std::endl;
#else
        cout << "\tNo MPI support." << std::endl;
#endif
        return 0;
    }

#ifdef HAVE_MPI
    // Avoid MPI log file clobber mania
    if (enable_mpi) {
        stringstream ss;
        ss << getpid();
        size_t pos = log_file.find_last_of('.');
        log_file.insert(pos, ss.str());
        log_file.insert(pos, "-pid-");
    }
#endif

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

    // Dump the stack every time SIGHUP is received.
    // void prt_stack(int sig) { logger().error("Caught SIGHUP"); };
    auto prt_stack = [](int sig) { logger().error("Caught SIGHUP"); };
    signal(SIGHUP, prt_stack);

    // Log command-line args
    logger().info() << "moses version " << version_string;
    string cmdline = "Command line:";
    for (int i = 0; i < argc; ++i) {
         cmdline += " ";
         cmdline += argv[i];
    }
    logger().info(cmdline);

    char hname[256];
    gethostname(hname, 256);
    logger().info("hostname: %s", hname);

    // Init random generator.
    randGen().seed(rand_seed);

    // Convert include_only_ops_str to the set of actual operators to
    // ignore.
    vertex_set ignore_ops;
    if (vm.count(include_only_ops_str_opt.first.c_str())) {
        bool ignore_operators = false;
        for (const string& s : include_only_ops_str) {
            vertex v;
            if (builtin_str_to_vertex(s, v)) {
                if (!ignore_operators) {
                    ignore_ops = {id::plus, id::times, id::div,
                                  id::exp, id::log, id::sin, id::impulse};
                    ignore_operators = true;
                }
                ignore_ops.erase(v);
            } else not_recognized_combo_operator(s);
        }
    }

    // Convert ignore_ops_str to the set of actual operators to ignore.
    for (const string& s : ignore_ops_str) {
        vertex v;
        if (builtin_str_to_vertex(s, v))
            ignore_ops.insert(v);
        else not_recognized_combo_operator(s);
    }

    // Specifying linear regression also ignores div exp log sin.
    if (linear_regression) {
        ignore_ops.insert(id::div);
        ignore_ops.insert(id::exp);
        ignore_ops.insert(id::log);
        ignore_ops.insert(id::sin);
    }

    // Set the initial exemplars.
    vector<combo_tree> exemplars;
    boost::transform(exemplars_str, std::back_inserter(exemplars),
                     str_to_combo_tree);

    // Fill jobs
    jobs_t jobs{{localhost, 1}}; // by default the localhost has 1 job
    bool local = true;
    for (const string& js : jobs_str) {
        size_t pos = js.find(job_seperator);
        if (pos != string::npos) {
            unsigned int nj = boost::lexical_cast<unsigned int>(js.substr(0, pos));
            string host_name = js.substr(pos + 1);
            jobs[host_name] = nj;
            local = false;
        } else {
            jobs[localhost] = boost::lexical_cast<unsigned int>(js);
        }
    }

    setting_omp(jobs[localhost], min_pool);

#ifdef HAVE_MPI
    if (enable_mpi) {
        if (!local) {
            logger().error("MPI-bsed distributed processing should not "
                "be mixed with host-based distributed processing!  "
                "That is: specify -j:hostname or --mpi but not both.\n");
            exit(1);
        }
        local = false;
        logger().info("Will run MOSES with MPI distributed processing.\n");
    }
#else
    logger().warn("WARNING: This version of MOSES does NOT have MPI support!\n");
    enable_mpi = false;
#endif

    // Set parameter structures. Please don't systematically use their
    // constructors (if you can avoid it), as it is prone to error,
    // whenever the constructor changes it may silently set the wrong
    // things (as long as they have the same types).

    // set feature selection parameters
    if (fs_focus == focus_all) {
        festor_params.restrict_incorrect = false;
        festor_params.restrict_true = false;
    } else if (fs_focus == focus_active) {
        festor_params.restrict_incorrect = false;
        festor_params.restrict_true = true;
    } else if (fs_focus == focus_incorrect) {
        festor_params.restrict_incorrect = true;
        festor_params.restrict_true = false;
    } else if (fs_focus == focus_ai) {
        festor_params.restrict_incorrect = true;
        festor_params.restrict_true = true;
    }

    if (fs_seed == seed_none) {
        festor_params.increase_target_size = false;
        festor_params.ignore_xmplr_features = true;
        festor_params.init_xmplr_features = false;
        festor_params.xmplr_as_feature = false;
    } else if (fs_seed == seed_add) {
        festor_params.increase_target_size = true;
        festor_params.ignore_xmplr_features = false;
        festor_params.init_xmplr_features = false;
        festor_params.xmplr_as_feature = false;
    } else if (fs_seed == seed_init) {
        festor_params.increase_target_size = false;
        festor_params.ignore_xmplr_features = false;
        festor_params.init_xmplr_features = true;
        festor_params.xmplr_as_feature = false;
    } else if (fs_seed == seed_xmplr) {
        festor_params.increase_target_size = false;
        festor_params.ignore_xmplr_features = false;
        festor_params.init_xmplr_features = false;
        festor_params.xmplr_as_feature = true;
    }

    // Set metapopulation parameters
    metapop_parameters meta_params;
    meta_params.max_candidates = max_candidates;
    meta_params.reduce_all = reduce_all;
    meta_params.revisit = revisit;
    meta_params.keep_bscore = output_bscore;
    meta_params.complexity_temperature = complexity_temperature;
    meta_params.cap_coef = cap_coef;
    meta_params.ignore_ops = ignore_ops;
    // meta_params.enable_cache = enable_cache;   // adaptive_cache
    meta_params.cache_size = cache_size;          // is disabled
    meta_params.jobs = jobs[localhost];
    meta_params.linear_contin = linear_regression;
    meta_params.perm_ratio = perm_ratio;

    // diversity parameters
    meta_params.diversity.include_dominated = include_dominated;
    meta_params.diversity.pressure = diversity_pressure;
    meta_params.diversity.exponent = diversity_exponent;
    meta_params.diversity.normalize = diversity_normalize;
    // set distance
    diversity_parameters::dst_enum_t de;
    if (diversity_dst == p_norm)
        de = diversity_parameters::p_norm;
    else if (diversity_dst == tanimoto)
        de = diversity_parameters::tanimoto;
    else if (diversity_dst == angular)
        de = diversity_parameters::angular;
    else {
        not_recognized_dst(diversity_dst);
        de = diversity_parameters::p_norm; // silent compiler warning
    }
    meta_params.diversity.set_dst(de, diversity_p_norm);
    // set distance to penalty
    diversity_parameters::dst2dp_enum_t d2de;
    if (diversity_dst2dp == auto_str)
        d2de = diversity_dst == p_norm ?
            diversity_parameters::inverse : diversity_parameters::complement;
    else if (diversity_dst2dp == inverse_str)
        d2de = diversity_parameters::inverse;
    else if (diversity_dst2dp == complement_str)
        d2de = diversity_parameters::complement;
    else if (diversity_dst2dp == power_str)
        d2de = diversity_parameters::pthpower;
    else {
        not_recognized_dst2dp(diversity_dst2dp);
        d2de = diversity_parameters::inverse; // silent compiler warning
    }
    meta_params.diversity.set_dst2dp(d2de);

    // Set optim_parameters.
    optim_parameters opt_params(opt_algo, pop_size_ratio, max_score, max_dist);
    hc_parameters hc_params;
    hc_params.widen_search = hc_widen_search;
    hc_params.single_step = hc_single_step;
    hc_params.crossover = hc_crossover;
    hc_params.crossover_pop_size = hc_crossover_pop_size;
    hc_params.max_nn_evals = hc_max_nn;
    hc_params.fraction_of_nn = hc_frac_of_nn;
    hc_params.allow_resize_deme = hc_allow_resize_deme;
    hc_params.prefix_stat_deme = "Demes";

    // Set moses_parameters.
    moses_parameters moses_params(vm, jobs);
    moses_params.local = local;
    moses_params.mpi = enable_mpi;
    moses_params.max_evals = max_evals;
    moses_params.max_gens = max_gens;
    moses_params.max_score = max_score;
    moses_params.max_time = max_time;
    moses_params.max_cnd_output = result_count;

    // Infer arity
    combo::arity_t arity = infer_arity(problem, problem_size, combo_str);

    // Input data for table-based problems.
    vector<Table> tables;
    vector<CTable> ctables;
    vector<string> ilabels;     // labels of the input table (table.itable)

    // Problem based on input table.
    if (datafile_based_problem(problem))
    {
        if (input_data_files.empty())
            no_input_datafile_exit();

        // Read input data files
        size_t num_rows = 0;
        for (const string& idf : input_data_files) {
            logger().info("Read data file %s", idf.c_str());
            Table table = loadTable(idf, target_feature, ignore_features_str);
            num_rows += table.size();
            // possible subsample the table
            if (nsamples > 0)
                subsampleTable(table, nsamples);
            tables.push_back(table);
            ctables.push_back(table.compressed());
        }
        logger().info("Number of rows in tables = %d", num_rows);

        // Get the labels contained in the data file.
        if (output_with_labels)
            ilabels = tables.front().itable.get_labels();

        arity = tables.front().get_arity();

        // Check that all input data files have the same arity
        if (tables.size() > 1) {
            combo::arity_t test_arity;
            for (size_t i = 1; i < tables.size(); ++i) {
                test_arity = tables[i].get_arity();
                if (test_arity != arity) {
                    not_all_same_arity_exit(input_data_files[0], arity,
                                            input_data_files[i], test_arity);
                }
            }
        }
    }

    logger().info("Inferred arity = %d", arity);

    // Set metapop printer parameters.
    metapop_printer mmr_pa(result_count,
                           output_score,
                           output_penalty,
                           output_bscore,
                           output_only_best,
                           output_eval_number,
                           output_with_labels,
                           ilabels,
                           output_file,
                           output_python,
                           enable_mpi);

    // Continuous reduction rules used during search and representation
    // building.
    const rule& contin_reduct = contin_reduction(reduct_candidate_effort,
                                                 ignore_ops);

    // Logical reduction rules used during search.
    logical_reduction r(ignore_ops);
    const rule& bool_reduct = r(reduct_candidate_effort);

    // Logical reduction rules used during representation building.
    const rule& bool_reduct_rep = r(reduct_knob_building_effort);

problem_params pms(bool_reduct, bool_reduct_rep, contin_reduct, moses_params, mmr_pa);
pms.enable_feature_selection = enable_feature_selection;
pms.nsamples = nsamples;
pms.arity = arity;
pms.ignore_ops = ignore_ops;
pms.it_abs_err = it_abs_err;
pms.noise = noise;
pms.complexity_ratio = complexity_ratio;
pms.exemplars = exemplars;
pms.opt_params = opt_params;
pms.hc_params = hc_params;
pms.meta_params = meta_params;

    register_demo_problems();

    typedef boost::ptr_vector<bscore_base> BScorerSeq;

    // Problem based on input table.
    if (datafile_based_problem(problem))
    {
        // Infer the signature based on the input table.
        const type_tree& table_type_signature = tables.front().get_signature();
        logger().info() << "Inferred data signature " << table_type_signature;

        // 'it' means regression based on input table; we maximimze accuracy.
        // 'pre' means we must maximize precision (i.e minimize the number of
        // false positives) while holding activation fixed.
        // 'prerec' means we must maximize precision (i.e minimize the number of
        // false positives) while holding recall fixed.
        // 'recall' means we must maximize recall while holding precision fixed.
        // 'f_one' means we must maximize the f_one score while holding ratio const
        // 'bep' means we must maximize the break-even point while holding difference const
        if (problem == it || problem == pre || problem == pre_conj ||
            problem == prerec || problem == recall ||
            problem == f_one || problem == bep)
        {
            // Infer the type of the input table
            type_tree table_output_tt = get_signature_output(table_type_signature);
            type_node table_output_tn = get_type_node(table_output_tt);

            // Determine the default exemplar to start with
            // problem == pre  precision-based scoring
            // precision combo trees always return booleans.
            if (exemplars.empty())
                exemplars.push_back(type_to_exemplar(
                    problem == pre? id::boolean_type : table_output_tn));

            // XXX This seems strange to me .. when would the exemplar output
            // type ever differ from the table?  Well,, it could for pre problem,
            // but that's over-ridden later, anyway...
            type_node output_type =
                get_type_node(get_output_type_tree(*exemplars.begin()->begin()));
            if (output_type == id::unknown_type)
                output_type = table_output_tn;

            logger().info() << "Inferred output type: " << output_type;

// Generic table regression code.  Could be a template, I suppose, but
// the args are variable length, tables is a variable, and scorer is a type,
// and I don't feel like fighting templates to make all three happen just
// exactly right.
#define REGRESSION(OUT_TYPE, REDUCT, REDUCT_REP, TABLES, SCORER, ARGS) \
{                                                                \
    /* Enable feature selection while selecting exemplar */      \
    if (enable_feature_selection && fs_params.target_size > 0) { \
        /* XXX FIXME: should use the concatenation of all */     \
        /* tables, and not just the first. */                    \
        meta_params.fstor = new feature_selector(TABLES.front(), festor_params); \
    }                                                            \
    /* Keep the table input signature, just make sure */         \
    /* the output is the desired type. */                        \
    type_tree cand_sig = gen_signature(                          \
        get_signature_inputs(table_type_signature),              \
        type_tree(OUT_TYPE));                                    \
    int as = alphabet_size(cand_sig, ignore_ops);                \
    typedef SCORER BScore;                                       \
    BScorerSeq bscores;                                          \
    for (const auto& table : TABLES) {                           \
        BScore* r = new BScore ARGS ;                            \
        set_noise_or_ratio(*r, as, noise, complexity_ratio);     \
        bscores.push_back(r);                                    \
    }                                                            \
    multibscore_based_bscore bscore(bscores);                    \
    metapop_moses_results(exemplars, cand_sig,                   \
                          REDUCT, REDUCT_REP, bscore,            \
                          opt_params, hc_params, meta_params, moses_params, \
                          mmr_pa);                               \
}
 
            // problem == pre  precision-based scoring
            if (problem == pre) {

                // Very nearly identical to the REGRESSION macro above,
                // except that some new experimental features are being tried.

                // Keep the table input signature, just make sure the
                // output is a boolean.
                type_tree cand_sig = gen_signature(
                    get_signature_inputs(table_type_signature),
                    type_tree(id::boolean_type));
                int as = alphabet_size(cand_sig, ignore_ops);
                typedef precision_bscore BScore;
                BScorerSeq bscores;
                for (const CTable& ctable : ctables) {
                    BScore* r = new BScore(ctable,
                                           fabs(hardness),
                                           min_rand_input,
                                           max_rand_input,
                                           hardness >= 0,
                                           pre_worst_norm);
                    set_noise_or_ratio(*r, as, noise, complexity_ratio);
                    bscores.push_back(r);
                    if (gen_best_tree) {
                        // experimental: use some canonically generated
                        // candidate as exemplar seed
                        combo_tree tr = r->gen_canonical_best_candidate();
                        logger().info() << "Canonical program tree (non reduced) maximizing precision = " << tr;
                        exemplars.push_back(tr);
                    }
                }

                // Enable feature selection while selecting exemplar
                if (enable_feature_selection && fs_params.target_size > 0) {
                    // XXX FIXME should use the concatenation of all ctables, not just first
                    meta_params.fstor = new feature_selector(ctables.front(),
                                                             festor_params);
                }

                multibscore_based_bscore bscore(bscores);
                metapop_moses_results(exemplars, cand_sig,
                                      bool_reduct, bool_reduct_rep, bscore,
                                      opt_params, hc_params, meta_params, moses_params,
                                      mmr_pa);
            }

            // problem == pre_conj  precision-based scoring (maximizing # conj)
            else if (problem == pre_conj) {

                // Very nearly identical to the REGRESSION macro above,
                // except that some new experimental features are being tried.

                // Keep the table input signature, just make sure the
                // output is a boolean.
                type_tree cand_sig = gen_signature(
                    get_signature_inputs(table_type_signature),
                    type_tree(id::boolean_type));
                int as = alphabet_size(cand_sig, ignore_ops);
                typedef precision_conj_bscore BScore;
                BScorerSeq bscores;
                for (const CTable& ctable : ctables) {
                    BScore* r = new BScore(ctable,
                                           fabs(hardness),
                                           hardness >= 0);
                    set_noise_or_ratio(*r, as, noise, complexity_ratio);
                    bscores.push_back(r);
                }

                // Enable feature selection while selecting exemplar
                if (enable_feature_selection && fs_params.target_size > 0) {
                    // XXX FIXME should use the concatenation of all ctables, not just first
                    meta_params.fstor = new feature_selector(ctables.front(),
                                                             festor_params);
                }

                multibscore_based_bscore bscore(bscores);
                metapop_moses_results(exemplars, cand_sig,
                                      bool_reduct, bool_reduct_rep, bscore,
                                      opt_params, hc_params, meta_params, moses_params,
                                      mmr_pa);
            }

            // problem == prerec  maximize precision, holding recall const.
            // Identical to above, just uses a different scorer.
            else if (problem == prerec) {
                if (0.0 == hardness) { hardness = 1.0; min_rand_input= 0.5;
                    max_rand_input = 1.0; }
                REGRESSION(id::boolean_type,
                           bool_reduct, bool_reduct_rep,
                           ctables, prerec_bscore,
                           (table, min_rand_input, max_rand_input, fabs(hardness)));
            }

            // problem == recall  maximize recall, holding precision const.
            else if (problem == recall) {
                if (0.0 == hardness) { hardness = 1.0; min_rand_input= 0.8;
                    max_rand_input = 1.0; }
                REGRESSION(id::boolean_type,
                           bool_reduct, bool_reduct_rep,
                           ctables, recall_bscore,
                           (table, min_rand_input, max_rand_input, fabs(hardness)));
            }

            // bep == beak-even point between recall and precision.
            else if (problem == bep) {
                if (0.0 == hardness) { hardness = 1.0; min_rand_input= 0.0;
                    max_rand_input = 0.5; }
                REGRESSION(id::boolean_type,
                           bool_reduct, bool_reduct_rep,
                           ctables, bep_bscore,
                           (table, min_rand_input, max_rand_input, fabs(hardness)));
            }

            // f_one = F_1 harmonic ratio of recall and precision
            else if (problem == f_one) {
                REGRESSION(id::boolean_type,
                           bool_reduct, bool_reduct_rep,
                           ctables, f_one_bscore,
                           (table));
            }

            // problem == it  i.e. input-table based scoring.
            else {
                OC_ASSERT(output_type == table_output_tn);

                // --------- Boolean output type
                if (output_type == id::boolean_type) {
                    REGRESSION(output_type,
                               bool_reduct, bool_reduct_rep,
                               ctables, ctruth_table_bscore,
                               (table));
                }

                // --------- Enumerated output type
                else if (output_type == id::enum_type) {

                    // For enum targets, like boolean targets, the score
                    // can never exceed zero (perfect score).
                    // XXX Eh ??? for precision/recall scorers,
                    // the score range is 0.0 to 1.0 so this is wrong...
                    if (0.0 < moses_params.max_score) {
                        moses_params.max_score = 0.0;
                        opt_params.terminate_if_gte = 0.0;
                    }
                    if (use_well_enough) {
                        // The "leave well-enough alone" algorithm.
                        // Works. Kind of. Not as well as hoped.
                        // Might be good for some problem. See diary.
                        partial_solver well(ctables,
                                        table_type_signature,
                                        exemplars, contin_reduct,
                                        opt_params, hc_params, meta_params,
                                        moses_params, mmr_pa);
                        well.solve();
                    } else {
                        // Much like the boolean-output-type above,
                        // just uses a slightly different scorer.
                        REGRESSION(output_type,
                                   contin_reduct, contin_reduct,
                                   ctables, enum_effective_bscore,
                                   (table));
                    }
                }

                // --------- Contin output type
                else if (output_type == id::contin_type) {
                    if (discretize_thresholds.empty()) {

                        contin_bscore::err_function_type eft =
                            it_abs_err ? contin_bscore::abs_error :
                            contin_bscore::squared_error;

                        REGRESSION(output_type,
                                   contin_reduct, contin_reduct,
                                   tables, contin_bscore,
                                   (table, eft));

                    } else {
                        REGRESSION(output_type,
                                   contin_reduct, contin_reduct,
                                   tables, discretize_contin_bscore,
                                   (table.otable, table.itable,
                                        discretize_thresholds, weighted_accuracy));
                    }
                }

                // --------- Unknown output type
                else {
                    unsupported_type_exit(output_type);
                }
            }
        }

        // Find interesting predicates
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
            BScorerSeq bscores;
            for (const CTable& ctable : ctables) {
                BScore *r = new BScore(ctable,
                                       ip_kld_weight,
                                       ip_skewness_weight,
                                       ip_stdU_weight,
                                       ip_skew_U_weight,
                                       min_rand_input,
                                       max_rand_input,
                                       hardness, hardness >= 0);
                set_noise_or_ratio(*r, as, noise, complexity_ratio);
                bscores.push_back(r);
            }
            multibscore_based_bscore bscore(bscores);
            metapop_moses_results(exemplars, tt,
                                  bool_reduct, bool_reduct_rep, bscore,
                                  opt_params, hc_params, meta_params, moses_params,
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
                                  opt_params, hc_params, meta_params, moses_params, mmr_pa);
        }
    }

    // Demo/Example: Problem based on input combo program.
    // Learn a program that should be identical to the specified input
    // program.
    else if (combo_based_problem(problem))
    {
        if (enable_feature_selection)
            logger().warn("Feature selection is not supported for that problem");

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

            type_node output_type = get_type_node(get_signature_output(tt));

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
                                      opt_params, hc_params, meta_params, moses_params,
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
                                      opt_params, hc_params, meta_params, moses_params,
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
                                  opt_params, hc_params, meta_params, moses_params, mmr_pa);
        }
    }

    // Demo/Example: learn a combo program that determines if the
    // program inputs are even parity or not.  That is, the combo
    // program will be a boolean circuit that computes parity.
    else if (problem == pa)
    {
        if (enable_feature_selection)
            logger().warn("Feature selection is not supported for that problem");

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
                              opt_params, hc_params, meta_params, moses_params, mmr_pa);
    }
    else {
        problem_base* probm = find_problem(problem);
        if (probm)
        {
            probm->run(pms);
        }
        else
            unsupported_problem_exit(problem);
    }
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
