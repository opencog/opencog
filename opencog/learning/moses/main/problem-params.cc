/**
 * problem-params.cc ---
 *
 * Copyright (C) 2010 OpenCog Foundation
 * Copyright (C) 2012, 2013 Poulin Holdings LLC
 * Copyright (C) 2013 Linas Vepstas
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

#include <boost/algorithm/string/trim.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

#include <opencog/util/oc_omp.h>

#include <opencog/comboreduct/table/table.h>
#include <opencog/comboreduct/table/table_io.h>

#include "moses_exec_def.h"
#include "problem-params.h"

namespace opencog { namespace moses {

using namespace std;

// diversity distance types
static const string p_norm = "p_norm";
static const string tanimoto = "tanimoto";
static const string angular = "angular";

// diversity dst2dp types
static const string auto_str = "auto";
static const string inverse_str = "inverse";
static const string complement_str = "complement";
static const string power_str = "power";

// Time bscore granularity
static const string day_str = "day";
static const string month_str = "month";

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

static void log_output_error_exit(string err_msg) {
    logger().error() << "Error: " << err_msg;
    cerr << "Error: " << err_msg << endl;
    exit(1);
}

/**
 * Display error message about not recognized combo operator and exist
 */
static void not_recognized_combo_operator(const string& ops_str)
{
    stringstream ss;
    ss << ops_str << " is not recognized as combo operator.";
    log_output_error_exit(ss.str());
}

//* Convert string to a combo_tree
static combo_tree str_to_combo_tree(const string& combo_str)
{
    stringstream ss;
    combo_tree tr;
    ss << combo_str;
    ss >> tr;
    return tr;
}

/**
 * Display error message about not recognized diversity distance and exist
 */
static void not_recognized_dst(const string& diversity_dst)
{
    std::stringstream ss;
    ss << diversity_dst << " is not recognized. Valid distances are "
       << p_norm << ", " << tanimoto << " and " << angular;
    log_output_error_exit(ss.str());
}

/**
 * Display error message about not recognized diversity distance to
 * penalty function and exist
 */
static void not_recognized_dst2dp(const string& diversity_dst2dp)
{
    stringstream ss;
    vector<string> valid_dsts = {auto_str, inverse_str, complement_str, power_str};
    ostreamContainer(ss << diversity_dst2dp
                     << " is not recognized. Valid distances to penalty are ",
                     valid_dsts, ", ");
    log_output_error_exit(ss.str());
}

problem_params::problem_params() :
    enable_mpi(false),
    default_nsamples(20),
    fs_params(festor_params.fs_params),
    max_filename_size(255)
{
}

void
problem_params::add_options(boost::program_options::options_description& desc)
{
    namespace po = boost::program_options;
    using boost::format;
    using namespace std;

    // Declare the supported options.
    // XXX TODO: make this print correctly, instead of using brackets.
    desc.add_options()

        // General options
        ("version", "Display the version of moses.\n")

        (opt_desc_str(jobs_opt).c_str(),
         po::value<vector<string>>(&jobs_str),
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

        ("min-pool",
         po::value<unsigned>(&min_pool)->default_value(50),
         "Minimum number of elements to process in the pool to enable "
         "multi-threading.\n")

        (opt_desc_str(exemplars_str_opt).c_str(),
         po::value<vector<string>>(&exemplars_str),
         "Start the search with a given exemplar, can be used several times.\n")

        // Problem-type options
        (opt_desc_str(problem_opt).c_str(),
         po::value<string>(&problem)->default_value("it"),
         "Problem to solve, supported problems are:\n\n"
         "it, regression based on input table\n\n"
         "pre, regression based on input table, maximizing precision, while holding activation fixed\n\n"
         "prerec, regression based on input table, maximizing precision, while holding recall fixed\n\n"
         "recall, regression based on input table, maximizing recall while holding precision fixed\n\n"
         "bep, regression based on input table, maximizing break-even point (BEP) between precision and recall\n\n"
         "f_one, regression based on input table, maximizing the F_1 score (harmonic mean of precision and recall)\n\n"
         "select, regression based on input table, selecting a range of rows\n\n"
         "ip, search interesting patterns, where interestingness"
         " is defined in terms of several features such as maximizing"
         " the Kullback-Leibler"
         " divergence between the distribution of the outputs and"
         " that same distribution in the context of the pattern"
         " being true."
         " Or the difference of skewnesses between the 2 distributions"
         " and other things being experimented.\n\n"
         "ann-it, regression based on input table using ann\n\n"
         "cp, demo, regression based on combo program\n\n"
         "pa, demo, even parity problem\n\n"
         "dj, demo, disjunction problem\n\n"
         "mux, demo, multiplex problem\n\n"
         "maj, demo, majority problem\n\n"
         "sr, demo, regression of f_n(x) = sum_{k=1,n} x^k\n")

        (opt_desc_str(nsamples_opt).c_str(),
         po::value<int>(&nsamples)->default_value(-1),
         "Number of samples to describe the problem. "
         "If nsample is negative, null or larger than the maximum "
         "number of samples allowed it is ignored. If the default "
         "problem size is larger than the value provided with that "
         "option then the dataset is subsampled randomly to reach the "
         "target size.\n")

        ("balance",
         po::value<bool>(&balance)->default_value(0),
         "If the table has discrete output type (like bool or enum), "
         "balance the resulting ctable so all classes have the same "
         "weight.\n")

        // Algorithm control options

        (opt_desc_str(opt_algo_opt).c_str(),
         po::value<string>(&opt_algo)->default_value(hc),
         str(format("Optimization algorithm, supported algorithms are "
                    "univariate (%s), simulation annealing (%s), "
                    "hillclimbing (%s). Of these, only hillclimbing "
                    "works well; the other algos have bit-rotted.\n")
             % un % sa % hc).c_str())

        (opt_desc_str(max_score_opt).c_str(),
         po::value<score_t>(&max_score)->default_value(very_best_score),
         "The max score to reach, once reached MOSES halts. If the largest "
         "floating point number is used and MOSES is able to calculate the "
         "max score that can be reached for that particular problem, "
         "it will overwrite it. "
         "Otherwise, for any other value, the user's defined max-score will "
         "be used.\n")

        (opt_desc_str(max_evals_opt).c_str(),
         po::value<unsigned long>(&max_evals)->default_value(10000),
         "Maximum number of fitness function evaluations.\n")

        ("max-time",
         po::value<time_t>(&max_time)->default_value(INT_MAX),
         "Longest allowed runtime, in seconds.\n")

        (opt_desc_str(cache_size_opt).c_str(),
         po::value<unsigned>(&cache_size)->default_value(3000),
         "Cache size. Memoize, that is, cache evaluation results, "
         "so that identical candidates are not re-evaluated.\n")
         // adaptive_cache has been temporarly disabled because it is
         // not thread safe, so the following comment doesn't apply
         // " The cache size is determined by this option "
         // "adjusted to fit in the RAM.\n")

        (opt_desc_str(ignore_ops_str_opt).c_str(),
         po::value<vector<string>>(&ignore_ops_str),
         str(format("Ignore the following operator in the program solution.  "
                    "This option may be used several times.  Currently, only div, "
                    "sin, exp, log can be ignored. "
                    "This option has the priority over --%s. "
                    "That is, if an operator is both be included and ignored, "
                    "then it is ignored.  This option does not work with ANN.\n")
             % include_only_ops_str_opt.first).c_str())

        ("linear-regression",
         po::value<bool>(&linear_regression)->default_value(false),
         "When attempting to fit continous-valued features, restrict "
         "searches to linear expressions only; that is, do not use "
         "polynomials in the fit.  Specifying this option also "
         "automatically disables the use of div, sin, exp and log.\n")

        ("logical-perm-ratio",
         po::value<double>(&perm_ratio)->default_value(0.0),
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
         po::value<unsigned long>(&rand_seed)->default_value(1),
         "Random seed.\n")

        (opt_desc_str(complexity_temperature_opt).c_str(),
         po::value<score_t>(&complexity_temperature)->default_value(6.0),
         "Set the \"temperature\" (scritly positive floating number) "
         "of the Boltzmann-like distribution "
         "used to select the next exemplar out of the metapopulaton. "
         "A temperature that is too high or too low will make it likely "
         "that poor exemplars will be chosen for exploration, thus "
         "resulting in excessively long search times. See the man "
         "page for more info.\n")

        (opt_desc_str(complexity_ratio_opt).c_str(),
         po::value<score_t>(&complexity_ratio)->default_value(3.5),
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
         "determined by earlier runs). See the man page for more info.\n")

        ("cap-coef",
         po::value<double>(&cap_coef)->default_value(50.0),
         "Set the leading coefficient of the formula defining the "
         "metapop size cap = cap_coef*(x+250)*(1+2*exp(-x/500)), "
         "where x is the number of generations so far. The default usually works "
         "well but if you run out of memory you may decrease that value.\n")

        // Large problem parameters
        ("hc-max-nn-evals",
         po::value<unsigned>(&hc_max_nn)->default_value(20000),
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
         po::value<double>(&hc_frac_of_nn)->default_value(2.0),
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

        (opt_desc_str(hc_crossover_opt).c_str(),
         po::value<bool>(&hc_crossover)->default_value(true),
         str(format("Hillclimbing crossover (%s). If false, then only "
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
         po::value<unsigned>(&hc_crossover_pop_size)->default_value(120),
         "Number of new candidates created by crossover during each iteration "
         "of hillclimbing.\n")

        ("hc-crossover-min-neighbors",
         po::value<unsigned>(&hc_crossover_min_neighbors)->default_value(400),
         "It also allows to control when crossover occurs instead of "
         "exhaustive search. If the neighborhood to explore has more than "
         "the given number (and at least 2 iterations has passed) then "
         "crossover kicks in.\n")

        ("hc-resize-to-fit-ram",
         po::value<bool>(&hc_resize_to_fit_ram)->default_value(false),
         str(format("Hillclimbing parameter (%s). If true, then the deme "
                    "is resized to fit in available memory. Note that "
                    "since it depends on the size of the installed RAM, "
                    "it will possibly introduce indeterminism between "
                    "runs on machines with different amounts of RAM.\n") % hc).c_str())

        // Same as above, old, deprecated name.
        ("hc-allow-resize-deme",
         po::value<bool>(&hc_resize_to_fit_ram)->default_value(false),
         str(format("Hillclimbing parameter (%s). If true, then the deme "
                    "is resized to fit in available memory. Note that "
                    "since it depends on the size of the installed RAM, "
                    "it will possibly introduce indeterminism between "
                    "runs on machines with different amounts of RAM.\n") % hc).c_str())

        // Algorithm tuning options
        ("boost",
         po::value<bool>(&boosting)->default_value(false),
         "Enable boosting for supervised learning problems.\n")

        ("boost-promote",
         po::value<int>(&num_to_promote)->default_value(1),
         "When boosting is enabled, this sets the number of candidates "
         "that, after every deme expansion cycle, should be added to "
         "the boosted ensemble.")

        ("boost-exact",
         po::value<bool>(&exact_experts)->default_value(true),
         "When boosting is enabled with the -Hpre table problem, this "
         "determines whether the expert candidates admitted into the "
         "ensemble must be perfect, exact experts, or if they can make "
         "mistakes.")

        ("boost-expalpha",
         po::value<double>(&expalpha)->default_value(2.0),
         "When boosting is enabled with the -Hpre table problem, and "
         "if the experts must be exact (option above), then this "
         "determines the ad-hoc weighting that magnifies unselected "
         "items in the dataset. ")

        ("boost-bias",
         po::value<double>(&bias_scale)->default_value(1.0),
         "When boosting is enabled with the -Hpre table problem, and "
         "if the experts are not exact, then a bias is used to distinguish "
         "the correctly selected and non-selected results.  This scale "
         "factor multiplies that bias. Best values are probably a bias "
         "of less than one.")

        (opt_desc_str(reduct_knob_building_effort_opt).c_str(),
         po::value<int>(&reduct_knob_building_effort)->default_value(2),
         "Effort allocated for reduction during knob building, 0-3, "
         "0 means minimum effort, 3 means maximum effort. The bigger "
         "the effort the lower the dimension of the deme.\n")

        (opt_desc_str(max_dist_opt).c_str(),
         po::value<size_t>(&max_dist)->default_value(4),
         "The maximum radius of the neighborhood around the "
         "exemplar to explore. This value only has an effect if "
         "the widen-search option has been set; this controls how "
         "far the search is widened.\n")

        (opt_desc_str(reduce_all_opt).c_str(),
         po::value<bool>(&reduce_all)->default_value(true),
         "Reduce all candidates before being evaluated.  Otherwise "
         "they are only reduced before being added to the "
         "metapopulation. This option can be valuable if memoization "
         "is enabled to avoid re-evaluate of duplicates.\n")

        (opt_desc_str(reduct_candidate_effort_opt).c_str(),
         po::value<int>(&reduct_candidate_effort)->default_value(2),
         "Effort allocated for reduction of candidates, in the range 0-3. "
         "0 means minimum effort, 3 means maximum effort.\n")

        (opt_desc_str(max_gens_opt).c_str(),
         po::value<int>(&max_gens)->default_value(-1),
         "Maximum number of demes to generate and optimize, "
         "negative means no generation limit.\n")

        ("discard-dominated",
         po::value<bool>(&discard_dominated)->default_value(false),
         "Include dominated candidates (according behavioral score) "
         "when merging candidates in the metapopulation. Disabling "
         "this may lead to poorer performance.\n")

        (opt_desc_str(hc_single_step_opt).c_str(),
         po::value<bool>(&hc_single_step)->default_value(false),
         str(format("Hillclimbing parameter (%s). If false, then the normal "
                    "hillclimbing algorithm is used.  If true, then only one "
                    "step is taken towards the hilltop, and the results are "
                    "promptly folded back into the metapopulation. If this "
                    "flag is set, then consider using the widen-search flag "
                    "as well, so as to make forward progress.\n") % hc).c_str())

        (opt_desc_str(include_only_ops_str_opt).c_str(),
         po::value<vector<string>>(&include_only_ops_str),
         "Include this operator, but exclude others, in the solution.  "
         "This option may be used several times to specify multiple "
         "operators.  Currently, only these operators are "
         "supported: plus, times, div, sin, exp, log. "
         "This option does not work with ANN.\n")

        (opt_desc_str(pop_size_ratio_opt).c_str(),
         po::value<double>(&pop_size_ratio)->default_value(20),
         "The higher the more effort is spent on a deme.\n")

        (opt_desc_str(noise_opt).c_str(),
         po::value<double>(&noise)->default_value(-1),
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
         po::value<bool>(&hc_widen_search)->default_value(false),
         str(format("Hillclimbing parameter (%s). If false, then deme search "
                    "terminates when a local hilltop is found. If true, "
                    "then the search radius is progressively widened, "
                    "until another termination condition is met. "
                    "The max-dist value controls just how wide the "
                    "search will go. The recommended setting is to "
                    "not perform widening.\n") % hc).c_str())

        ("well-enough",
         po::value<bool>(&use_well_enough)->default_value(false),
         "If 1, use the \"leave well-enough alone\" algorithm for "
         "classification problems. This algorithm, after finding a "
         "clause that has perfect accuracy, will stop mutating that "
         "clause any further. In principle, this should speed "
         "convergence.  In practice, not so much; it can hurt "
         "performance.\n")

        ("revisit",
         po::value<int>(&revisit)->default_value(0),
         "Number of times the same exemplar can be revisited. "
         "This option is only worthwhile when there "
         "is a great deal of stochasticity in the search so that exploring "
         "a deme spawn from the same exemplar multiple times will yield "
         "substantially different results. If the value is negative then the "
         "number of revisit is unbound. "
         "This might be the case is feature selection is used for instance.\n")

        // Output control options

        (opt_desc_str(result_count_opt).c_str(),
         po::value<long>(&result_count)->default_value(10),
         "The number of results to return, ordered according to "
         "a linear combination of score and complexity. If negative, "
         "then return all results.\n")

        (opt_desc_str(output_score_opt).c_str(),
         po::value<bool>(&output_score)->default_value(true),
         "If 1, output the score before each candidate\n")

        (opt_desc_str(output_cscore_opt).c_str(),
         po::value<bool>(&output_cscore)->default_value(false),
         "If 1, output the composite score after each candidate\n")

        (opt_desc_str(output_bscore_opt).c_str(),
         po::value<bool>(&output_bscore)->default_value(false),
         "If 1, output the bscore after each candidate (possibly after the composite score).\n")

        (opt_desc_str(output_only_best_opt).c_str(),
         po::value<bool>(&output_only_best)->default_value(false),
         "If 1, print only the best candidates.\n")

        (opt_desc_str(output_eval_number_opt).c_str(),
         po::value<bool>(&output_eval_number)->default_value(false),
         "If 1, output the actual number of evaluations.\n")

        (opt_desc_str(output_with_labels_opt).c_str(),
         po::value<bool>(&output_with_labels)->default_value(false),
         "If 1, output the candidates with argument labels "
         "instead of argument numbers. For instance "
         "*(\"$price\" \"$temperature\") instead of *($1 $2). This only "
         "works for data fitting problems where the data file contains "
         "labels in its header.\n")

        ("output-deme-id",
         po::value<bool>(&output_deme_id)->default_value(false),
         "If 1, output the deme ID where the candidates have been produced first. "
         "Deme 0 is the initial deme (before the first expansion). "
         "Specifically the ID follows the format "
         "EXPANSION[.BREADTH_FIRST_INDEX[.SUBSAMPLED_INDEX]].\n")

        ("python",
         po::value<bool>(&output_python)->default_value(false),
         "If 1, output the program(s) as python code instead of combo. "
         "Best with -c1 option to return a single python module. Only "
         "implemented for boolean programs currently.\n")

        (opt_desc_str(output_file_opt).c_str(),
         po::value<string>(&output_file)->default_value(""),
         "File where to place the output. If empty, then output to stdout.\n")

        // The remaining options (TODO organize this)

        (opt_desc_str(min_rand_input_opt).c_str(),
         po::value<double>(&min_rand_input)->default_value(0.0),
         "Minimum value of a sampled coninuous input.  The cp, ip, pre, "
         "recall, prerec, bep, f_one and select "
         "problems all require a range of values to be sampled in "
         "order to measure the fitness of a proposed solution. This "
         "option sets the low end of the sampled range. In the case of "
         "fitness function pre, the range corresponds to the activation "
         "of the precision.\n")

        (opt_desc_str(max_rand_input_opt).c_str(),
         po::value<double>(&max_rand_input)->default_value(1.0),
         "Maximum value of a sampled coninuous input.  The cp, ip, pre, "
         "recall, prerec, bep, f_one and select "
         "problems all require a range of values to be sampled in "
         "order to measure the fitness of a proposed solution. This "
         "option sets the high end of the sampled range. In the case of "
         "fitness function pre, the range corresponds to the activation "
         "of the precision.\n")

        (opt_desc_str(log_level_opt).c_str(),
         po::value<string>(&log_level)->default_value("INFO"),
         "Log level, possible levels are NONE, ERROR, WARN, INFO, "
         "DEBUG, FINE. Case does not matter.\n")

        (opt_desc_str(log_file_dep_opt_opt).c_str(),
         str(format("The name of the log is determined by the options, for"
                    " instance if moses-exec is called with -%s 123 -%s pa"
                    " the log name is moses_random-seed_123_problem_pa.log."
                    " The name will be truncated in order not to"
                    " be longer than %s characters.\n")
             % rand_seed_opt.second % problem_opt.second
             % max_filename_size).c_str())

        (opt_desc_str(log_file_opt).c_str(),
         po::value<string>(&log_file)->default_value(default_log_file),
         str(format("File name where to write the log."
                    " This option is overwritten by %s.\n")
             % log_file_dep_opt_opt.first).c_str())

        (opt_desc_str(max_candidates_opt).c_str(),
         po::value<int>(&max_candidates)->default_value(-1),
         "Maximum number of considered candidates to be added to the "
         "metapopulation after optimizing deme.\n")

#ifdef HAVE_MPI
        ("mpi",
         po::value<bool>(&enable_mpi)->default_value(false),
         "Enable MPI-based distributed processing.\n")
#endif

        (opt_desc_str(weighted_accuracy_opt).c_str(),
         po::value<bool>(&weighted_accuracy)->default_value(false),
         "This option is only used for the discretize_contin_bscore "
         "(when --discretize-threshold is used), "
         "if enabled, then the score corresponds to weighted accuracy"
         "Useful in case of unbalanced data.\n")

        ("diversity-pressure",
         po::value<score_t>(&diversity_pressure)->default_value(0.0),
         "Set a diversity pressure on the metapopulation. "
         "Programs behaving similarily to others are more penalized. "
         "That value sets the importance of that penalty (from 0 to +inf).\n")

        ("diversity-exponent",
         po::value<score_t>(&diversity_exponent)->default_value(-1.0),
         "Set the exponent of the generalized mean (or sum, if "
         "--diversity-normalize is set to 0) aggregating "
         "the penalties between a candidate and the set of all candidates better "
         "than itself (taking into account diversity). If the value tends "
         "towards 0 it tends to the geometric mean, towards +inf it tends "
         "to the max function. If negative or null is it the max function.\n")

        ("diversity-normalize",
         po::value<bool>(&diversity_normalize)->default_value(true),
         "If set to 1 then the aggregating function is a generalized mean. "
         "Otherwize it is a generalized sum (generalize mean * number of "
         "elements). If --diversity-exponent is set to negatively then "
         "this doesn't have any impact as the aggregating function is "
         "the max anyway.\n")

        ("diversity-dst",
         po::value<string>(&diversity_dst)->default_value(p_norm),
         str(format("Set the distance between behavioral scores, "
                    "then used to determin the diversity penalty."
                    "3 distances are available: %s, %s and %s.\n")
             % p_norm % tanimoto % angular).c_str())

        ("diversity-p-norm",
         po::value<score_t>(&diversity_p_norm)->default_value(2.0),
         "Set the parameter of the p_norm distance. A value of 1.0"
         "correspond to the Manhatan distance. A value of 2.0 corresponds to "
         "the Euclidean distance. A value of 0.0 or less correspond to the "
         "max component-wise. Any other value corresponds to the general case.\n")

        ("diversity-dst2dp",
         po::value<string>(&diversity_dst2dp)->default_value(auto_str),
         str(format("Set the type of function to convert distance into penalty. "
                    "4 options are available: %1%, %2%, %3% and %4%. "
                    "When %1% is selected the function is selected depending "
                    "on the distance, if the distance is %5%, "
                    "then %2% is selected, otherwise %3% is selected.\n")
             % auto_str % inverse_str % complement_str
             % power_str % p_norm).c_str())

        (opt_desc_str(discretize_threshold_opt).c_str(),
         po::value<vector<contin_t>>(&discretize_thresholds),
         "If the domain is continuous, discretize the target feature. "
         "A unique used of that option produces 2 classes, x < thresold "
         "and x >= threshold. The option can be used several times (n-1) "
         "to produce n classes and the thresholds are automatically sorted.\n")

        (opt_desc_str(alpha_opt).c_str(),
         po::value<score_t>(&hardness)->default_value(1.0),
         "If problems pre, prerec, recall, f_one, bep ore select are "
         "specified, "
         "this option is used to set the 'hardness' of the constraint, "
         "with larger values corresponding to a harder constraint "
         "(i.e. punishing the score more strongly if the contraint "
         "is not met.)  For the 'pre' problem, if alpha is negative, "
         "then its absolute value is used for the hardness, and "
         "the negative predictive value is maximized (instead of "
         "the precision).\n")

        ("time-dispersion-pressure",
         po::value<score_t>(&time_dispersion_pressure)->default_value(0.0),
         "Adds a penalty in the fitness to ensure that models have "
         "their activation spread across time.\n")

        ("time-dispersion-exponent",
         po::value<score_t>(&time_dispersion_exponent)->default_value(1.0),
         "Distort the penalty.\n")

        ("time-bscore",
         po::value<bool>(&time_bscore)->default_value(false),
         "In case the data has timestamp, spread the bscore across the "
         "timestamps instead of data points.\n")

        ("time-bscore-granularity",
         po::value<string>(&time_bscore_granularity_str)->default_value(day_str),
         "Set the granularity of timestamp, in case the bscore is spread "
         "across time. Options are 'day' and 'month'.\n")

        ("it-abs-err",
         po::value<bool>(&it_abs_err)->default_value(false),
         "Use absolute error instead of squared error [EXPERIMENTAL, the occam's razor hasn't been calibrated for that fitness function yet].\n")

        ("gen-best-tree",
         po::value<bool>(&gen_best_tree)->default_value(false),
         "Attempts to generate the best candidate (possibly huge and overfit) head-on. Only works combined with -Hpre for now.\n")

        // ======= Feature-selection params =======
        ("enable-fs",
         po::value<bool>(&enable_feature_selection)->default_value(false),
         "Enable integrated feature selection.  Feature selection is "
         "performed immediately before knob building (representation "
         "building), when creating a new deme.  Limiting the number "
         "of features can sharply improve the run-time and memory "
         "usage of large problems.\n")

        ("fs-target-size",
         po::value<unsigned>(&fs_params.target_size)->default_value(20),
         "Feature count.  This option "
         "specifies the number of features to be selected out of "
         "the dataset.  A value of 0 disables feature selection.\n")

        ("fs-exp-distrib",
         po::value<bool>(&fs_params.exp_distrib)->default_value(false),
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
         po::value<string>(&fs_focus)->default_value(focus_incorrect),
         str(boost::format("Focus of feature selection (which data points "
                           "feature will focus on):\n\n"
                           "%s, all data points are considered\n\n"
                           "%s, only active data points are considered\n\n"
                           "%s, only incorrect answers are considered\n\n"
                           "%s, only active data points that are incorrectly "
                           "answered are considered.\n")
             % focus_all % focus_active % focus_incorrect % focus_ai).c_str())

        ("fs-seed",
         po::value<string>(&fs_seed)->default_value(seed_add),
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
         po::value<bool>(&festor_params.prune_xmplr)->default_value(0),
         "Remove from the exemplar the literals of non-selected features.\n")

        ("fs-subsampling-ratio",
         po::value<double>(&festor_params.subsampling_ratio)->default_value(1),
         "Subsampling size ratio. 1 means no subsampling is taking place. "
         "0 means is the most extrem subsampling (all data are discarded). "
         "This is useful to introduce some randomness in "
         "feature selection, as not all feature selection algorithms "
         "have some.\n")

        ("fs-subsampling-by-time",
         po::value<bool>(&festor_params.subsampling_by_time)->default_value(0),
         "If enabled, then fs-subsampling-ratio applies to the set of timestamps "
         "rather than the set of rows. And only rows timestamped at subsampled "
         "timestamps are kept.\n")

        ("fs-demes",
         po::value<unsigned>(&festor_params.n_demes)->default_value(1),
         "Number of different feature sets to select during dynamic "
         "feature selection.  Each different feature set will be used "
         "to create a different deme for a single, given exemplar. By "
         "default, only a single deme is created. (XXX Does this "
         "actually improve anything? Or does this just add to the "
         "complexity of the code base??? Cause I don't see the utility "
         "of this.)\n")

        ("fs-algo",
         po::value<string>(&fs_params.algorithm)->default_value("simple"),
         string("Feature selection algorithm. Supported algorithms are:\n"
         "  simple, for a fast maximun mutual information algo.\n"
         "  inc, for incremental max-relevency, min-redundancy.\n"
         "  smd, for stochastic mutual dependency,\n"
         "  random, for uniform random dependency,\n")
         .append(moses::hc).append(" for moses-hillclimbing.\n").c_str())

        ("fs-scorer",
         po::value<string>(&fs_params.scorer)->default_value(mi),
         str(boost::format("Feature selection fitness function (scorer).\n"
                           " Supported scorers are:\n"
                           "%s, for mutual information\n"
                           "%s, for precision (see moses -h for more info)\n")
             % mi % pre).c_str())

        ("fs-threshold",
         po::value<double>(&fs_params.threshold)->default_value(0),
         "Improvement threshold. Floating point number. "
         "Specifies the threshold above which the mutual information "
         "of a feature is considered to be significantly correlated "
            "to the target.  A value of zero means that all features "
         "will be selected. \n"
         "For the -ainc algo only, the -C flag over-rides this setting.\n")

        ("fs-enforce-features-filename",
         po::value<string>(&fs_enforce_features_filename),
         "File containing a list (seperated by newline) of a feature name"
         "possibly followed by a weight between 0 and 1. If the weight is"
         "missing it is equivalent to being 1. The weight represent the"
         "probability of being of the feature of being inserted in the deme.\n")

        // ======= Feature-selection diversity pressure =======
        ("fs-diversity-pressure",
         po::value<double>(&festor_params.diversity_pressure)->default_value(0.0),
         "Multiplicative coefficient of the diversity penalty "
         "(itself being in [0,1]).\n")

        ("fs-diversity-cap",
         po::value<size_t>(&festor_params.diversity_cap)->default_value(100),
         "Place a cap on the maximum number of feature set to consider. "
         "If zero, no cap is used (Warning: could be very slow). "
         "Use this to speed up diversity computation on feature sets.\n")

        ("fs-diversity-interaction",
         po::value<int>(&festor_params.diversity_interaction)->default_value(-1),
         "Maximum number of interactions to be considered when computing "
         "the mutual information between feature sets. "
         "This is used in case the number of selected features tends to "
         "be high compared to the number of datapoints to decrease inacuracy "
         "of the mutual information.\n")

        ("fs-diversity-jaccard",
         po::value<bool>(&festor_params.diversity_jaccard)->default_value(true),
         "Instead of using the expensive mutual information between feature "
         "sets to measure diversity, it uses a cheap Jaccard index. In that "
         "case the feature semantic is ignored")

        // ======= Feature-selection incremental algo params =======
        ("fs-inc-redundant-intensity",
         po::value<double>(&fs_params.inc_red_intensity)->default_value(-1.0),
         "Incremental Selection parameter. Floating-point value must "
         "lie between 0.0 and 1.0.  A value of 0.0 or less means that no "
         "redundant features will discarded, while 1.0 will cause a "
         "maximal number will be discarded.\n")

        ("fs-inc-target-size-epsilon",
         po::value<double>(&fs_params.inc_target_size_epsilon)->default_value(1.0e-6),
         "Incremental Selection parameter. Tolerance applied when "
         "selecting for a fixed number of features (option -C).\n")

        ("fs-inc-interaction-terms",
         po::value<unsigned>(&fs_params.inc_interaction_terms)->default_value(1),
         "Incremental Selection parameter. Maximum number of "
         "interaction terms considered during incremental feature "
         "selection. Higher values make the feature selection more "
         "accurate but is combinatorially more computationally expensive.\n")

        // ======= Feature-selection pre scorer only params =======
        ("fs-pre-penalty",
         po::value<double>(&fs_params.pre_penalty)->default_value(1.0),
         "Activation penalty (see moses --help or man moses for more info).\n")

        ("fs-pre-min-activation",
         po::value<double>(&fs_params.pre_min_activation)->default_value(0.5),
         "Minimum activation (see moses --help or man moses for more info).\n")

        ("fs-pre-max-activation",
         po::value<double>(&fs_params.pre_max_activation)->default_value(1.0),
         "Maximum activation (see moses --help or man moses for more info).\n")

        ("fs-pre-positive",
         po::value<bool>(&fs_params.pre_positive)->default_value(true),
         "If 1, then precision, otherwise negative predictive value "
         "(see moses --help or man moses for more info).\n")

        // ======= Feature-selection hill-climbing only params =======
        ("fs-hc-max-score",
         po::value<double>(&fs_params.hc_max_score)->default_value(1),
         "Hillclimbing parameter.  The max score to reach, once "
         "reached feature selection halts.\n")

        // no need of that for now
        // (opt_desc_str(hc_initial_feature_opt).c_str(),
        //  po::value<vector<string> >(&fs_params.hc_initial_features),
        //  "Hillclimbing parameter.  Initial feature to search from.  "
        //  "This option can be used as many times as there are features, "
        //  "to have them included in the initial feature set. If the "
        //  "initial feature set is close to the one that maximizes the "
        //  "quality measure, the selection speed can be greatly increased.\n")

        ("fs-hc-max-evals",
         po::value<unsigned>(&fs_params.hc_max_evals)->default_value(10000),
         "Hillclimbing parameter.  Maximum number of fitness function "
         "evaluations.\n")

        ("fs-hc-fraction-of-remaining",
         po::value<double>(&fs_params.hc_fraction_of_remaining)->default_value(0.5),
         "Hillclimbing parameter.  Determine the fraction of the "
         "remaining number of eval to use for the current iteration.\n")

        ("fs-hc-crossover",
         po::value<bool>(&fs_params.hc_crossover)->default_value(false),
         "Hillclimber crossover (see --hc-crossover option)\n")

        ("fs-hc-crossover-pop-size",
         po::value<unsigned>(&fs_params.hc_crossover_pop_size)->default_value(120),
         "Hillclimber crossover pop size (see --hc-crossover option)\n")

        ("fs-hc-crossover-min-neighbors",
         po::value<unsigned>(&fs_params.hc_crossover_min_neighbors)->default_value(400),
         "Hillclimber crossover min neighbors (see --hc-crossover option)\n")

        ("fs-hc-widen-search",
         po::value<bool>(&fs_params.hc_widen_search)->default_value(true),
         "Hillclimber widen_search (see --widen-search)\n")

        // ======= Feature-selection MI scorer params =======
        ("fs-mi-penalty",
         po::value<double>(&fs_params.mi_confi)->default_value(100.0),
         "Mutual-information scorer parameter.  Intensity of the confidence "
         "penalty, in the range (-Inf, +Inf).  100 means no confidence "
         "penalty. This parameter influences how much importance is "
         "attributed to the confidence of the quality measure. The "
         "fewer samples in the data set, the more features the "
         "less confidence in the feature set quality measure.\n")

        // ======= Feature-selection SMD params =======
        ("fs-smd-top-size",
         po::value<unsigned>(&fs_params.smd_top_size)->default_value(10),
         "Stochastic max dependency parameter. Number of feature subset "
         "candidates to consider building the next superset.\n")

        // ======= Subsample-deme params =======
        ("ss-n-subsample-demes",
         po::value<unsigned>(&ss_n_subsample_demes)->default_value(0),
         "Number of demes to use for subsampling filter.\n")

        ("ss-n-top-candidates",
         po::value<unsigned>(&ss_n_top_candidates)->default_value(1),
         "Number of top candidates to consider to calculate the score standard "
         "deviation.\n")

        ("ss-n-tuples",
         po::value<unsigned>(&ss_n_tuples)->default_value(UINT_MAX),
         "Number of tuples used to calculate the estimate of the standard "
         "deviation of the top candidates. If the number of top candidates "
         "(set by --ss-n-top-candidates) is too high you might want to set "
         "that value lower than default to avoid a computational "
         "bottleneck.\n")

        ("ss-std-dev-threshold",
         po::value<float>(&ss_std_dev_threshold)->default_value(std::numeric_limits<float>::max()),
         "An average deme score standard deviation needs to fall under that value to"
         "be included in the metapopulation.\n")

        ("ss-tanimoto-mean-threshold",
         po::value<float>(&ss_tanimoto_mean_threshold)->default_value(1.0),
         "Demes are merged to the metapopulation only if the average tanimoto "
         "distances between the top candidates of "
         "the subsampled demes fall below that value.\n")

        ("ss-tanimoto-geometric-mean-threshold",
         po::value<float>(&ss_tanimoto_geo_mean_threshold)->default_value(1.0),
         "Demes are merged to the metapopulation only if the geometric average tanimoto "
         "distances between the top candidates of "
         "the subsampled demes fall below that value.\n")

        ("ss-tanimoto-max-threshold",
         po::value<float>(&ss_tanimoto_max_threshold)->default_value(1.0),
         "Demes are merged to the metapopulation only if the max tanimoto "
         "distances between the top candidates of "
         "the subsampled demes fall below that value.\n")

        ("ss-n-best-bfdemes",
         po::value<unsigned>(&ss_n_best_bfdemes)->default_value(0),
         "Alternate way to subsampling filter. "
         "Instead the n best breadth first demes are selected.\n")

        ("ss-tanimoto-mean-weight",
         po::value<float>(&ss_tanimoto_mean_weight)->default_value(0.0),
         "Weight to determine the aggregated tanimoto distance for "
         "--ss-n-best-bfdemes.\n")

        ("ss-tanimoto-geometric-mean-weight",
         po::value<float>(&ss_tanimoto_geo_mean_weight)->default_value(0.0),
         "Weight to determine the aggregated tanimoto distance for "
         "--ss-n-best-bfdemes.\n")

        ("ss-tanimoto-max-weight",
         po::value<float>(&ss_tanimoto_max_weight)->default_value(0.0),
         "Weight to determine the aggregated tanimoto distance for "
         "--ss-n-best-bfdemes.\n")

        ("ss-n-subsample-fitnesses",
         po::value<unsigned>(&ss_n_subsample_fitnesses)->default_value(0),
         "Number of subsampled fitnesses to use for low score deviation pressure. "
         "Ignored is 0 r 1.\n")

        ("ss-low-dev-pressure",
         po::value<float>(&ss_low_dev_pressure)->default_value(1.0),
         "How much low score deviation pressure there is.\n")

        ("ss-by-time",
         po::value<bool>(&ss_by_time)->default_value(0),
         "Subsample by time.\n")

        ("ss-contiguous-time",
         po::value<bool>(&ss_contiguous_time)->default_value(1),
         "If subsample by time is enable then subsample contiguous "
         "(chronologically ordered) time segments.\n")

        // ========== THE END of the options; note semicolon ===========
        ;

}

void problem_params::parse_options(boost::program_options::variables_map& vm)
{
    if (vm.count("version")) {
        cout << "moses " << version_string << std::endl;
#ifdef HAVE_MPI
        cout << "\tMPI support enabled." << std::endl;
#else
        cout << "\tNo MPI support." << std::endl;
#endif
        exit(0);
    }

    // set flags
    bool have_log_file_opt = vm.count(log_file_dep_opt_opt.first) > 0;

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
    if (have_log_file_opt) {
        std::set<std::string> ignore_opt{log_file_dep_opt_opt.first};
        log_file = determine_log_name(default_log_file_prefix,
                                      vm, ignore_opt,
                                      string(".").append(default_log_file_suffix));
    }

    // Remove old log_file before setting the new one.
    remove(log_file.c_str());
    logger().setFilename(log_file);
    boost::trim(log_level);
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

    // Log some generic, important information.
    logger().info() << "moses version " << version_string;
    char hname[256];
    gethostname(hname, 256);
    logger().info("hostname: %s", hname);

    // Init random generator.
    randGen().seed(rand_seed);

    // Convert include_only_ops_str to the set of actual operators to
    // ignore.
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

    // Parse enforce features
    if (!fs_enforce_features_filename.empty()) {
        ifstream in(fs_enforce_features_filename.c_str());
        OC_ASSERT(in.is_open(), "Could not open %s",
                  fs_enforce_features_filename.c_str());

        while (in) {
            string line;
            getline(in, line);
            if (line.empty())
                continue;

            // Check if there is a float (there must be a whiltespace then)
            auto whitespace_pos = line.find(' ');
            if (whitespace_pos != std::string::npos) {
                string feature_name = line.substr(0, whitespace_pos);
                float weight = stof(line.substr(whitespace_pos));
                festor_params.enforce_features[feature_name] = weight;
            } else {
                festor_params.enforce_features[line] = 1;
            }
        }

        for (auto& p : festor_params.enforce_features) {
            logger().debug() << "Enforce feature " << p.first
                             << " with probability " << p.second;
        }
    }

    // Set deme expansion paramters
    deme_params.reduce_all = reduce_all;
    deme_params.ignore_ops = ignore_ops;
    deme_params.linear_contin = linear_regression;
    deme_params.perm_ratio = perm_ratio;
    if (ss_n_subsample_demes > 1 or ss_n_subsample_fitnesses > 1) {
        // If SS-MOSES is enabled then the cache is automatically
        // disabled because SS-MOSES implies to re-evaluate the same
        // candidates over slightly different fitness functions
        logger().debug() << "Disable cache because subsampling is enabled";
        cache_size = 0;
    } else {
        // cache_size = cache_size;
    }

    // Set metapopulation parameters
    meta_params.max_candidates = max_candidates;
    meta_params.revisit = revisit;
    meta_params.do_boosting = boosting;
    meta_params.ensemble_params.do_boosting = boosting;
    meta_params.ensemble_params.num_to_promote = num_to_promote;
    meta_params.ensemble_params.exact_experts = exact_experts;
    meta_params.ensemble_params.expalpha = expalpha;
    meta_params.ensemble_params.bias_scale = bias_scale;
    if (boosting) cache_size = 0;  // cached cscores are stale!
    meta_params.discard_dominated = discard_dominated;
    meta_params.keep_bscore = output_bscore;
    meta_params.complexity_temperature = complexity_temperature;
    meta_params.cap_coef = cap_coef;
    meta_params.jobs = jobs[localhost];

    // diversity parameters
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

    // Set time bscore granularity
    if (time_bscore_granularity_str == day_str)
        time_bscore_granularity = TemporalGranularity::day;
    else if (time_bscore_granularity_str == month_str)
        time_bscore_granularity = TemporalGranularity::month;
    else {
        stringstream ss;
        ss << "Granularity " << time_bscore_granularity_str << " not implemented";
        log_output_error_exit(ss.str());
    }

    // Subsampling deme and fitness parameters
    auto& ss_params = filter_params;
    ss_params.n_subsample_demes = ss_n_subsample_demes;
    ss_params.n_top_candidates = ss_n_top_candidates;
    ss_params.n_tuples = ss_n_tuples;
    ss_params.std_dev_threshold = ss_std_dev_threshold;
    ss_params.tanimoto_mean_threshold = ss_tanimoto_mean_threshold;
    ss_params.tanimoto_geo_mean_threshold = ss_tanimoto_geo_mean_threshold;
    ss_params.tanimoto_max_threshold = ss_tanimoto_max_threshold;
    ss_params.n_best_bfdemes = ss_n_best_bfdemes;
    ss_params.tanimoto_mean_weight = ss_tanimoto_mean_weight;
    ss_params.tanimoto_geo_mean_weight = ss_tanimoto_geo_mean_weight;
    ss_params.tanimoto_max_weight = ss_tanimoto_max_weight;
    ss_params.n_subsample_fitnesses = ss_n_subsample_fitnesses;
    ss_params.low_dev_pressure = ss_low_dev_pressure;
    ss_params.by_time = ss_by_time;
    ss_params.contiguous_time = ss_contiguous_time;

    // Set optim_parameters.
    opt_params = optim_parameters(opt_algo, pop_size_ratio, max_score, max_dist);
    hc_params.widen_search = hc_widen_search;
    hc_params.single_step = hc_single_step;
    hc_params.crossover = hc_crossover;
    hc_params.crossover_pop_size = hc_crossover_pop_size;
    hc_params.crossover_min_neighbors = hc_crossover_min_neighbors;
    hc_params.max_nn_evals = hc_max_nn;
    hc_params.fraction_of_nn = hc_frac_of_nn;
    hc_params.resize_to_fit_ram = hc_resize_to_fit_ram;
    hc_params.prefix_stat_deme = "Demes";

    // Set moses_parameters.
    moses_params = moses_parameters(vm, jobs);
    moses_params.local = local;
    moses_params.mpi = enable_mpi;
    moses_params.max_evals = max_evals;
    moses_params.max_gens = max_gens;
    moses_params.max_score = max_score;
    moses_params.max_time = max_time;
    moses_params.max_cnd_output = result_count;

    // Logical reduction rules used during search.
    lr = reduct::logical_reduction(ignore_ops);
    bool_reduct = lr(reduct_candidate_effort).clone();

    // Logical reduction rules used during representation building.
    bool_reduct_rep = lr(reduct_knob_building_effort).clone();

    // Continuous reduction rules used during search and representation
    // building.
    contin_reduct = reduct::contin_reduction(reduct_candidate_effort, ignore_ops).clone();

    // Set metapop printer parameters.
    mmr_pa = metapop_printer(result_count,
                             output_score,
                             output_cscore,
                             output_bscore,
                             output_only_best,
                             boosting,
                             output_eval_number,
                             output_with_labels,
                             output_deme_id,
                             col_labels,
                             output_file,
                             output_python,
                             enable_mpi);

}

} // ~namespace moses
} // ~namespace opencog

