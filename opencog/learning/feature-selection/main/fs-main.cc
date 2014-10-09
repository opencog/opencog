/** main.cc ---
 *
 * Copyright (C) 2011 OpenCog Foundation
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

#include "feature-selection.h"

#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/range/algorithm/find.hpp>
#include <boost/format.hpp>

#include <opencog/util/log_prog_name.h>
#include <opencog/util/oc_omp.h>

#include <opencog/comboreduct/table/table_io.h>
#include <opencog/learning/moses/moses/moses_main.h> // for version string

using namespace std;
using namespace boost::program_options;
using namespace opencog;
using boost::lexical_cast;
using boost::trim;
using boost::str;

// Assorted defaults.
static const unsigned max_filename_size = 255;

static const string default_log_file_prefix = "feature-selection";
static const string default_log_file_suffix = "log";
static const string default_log_file = default_log_file_prefix + "." + default_log_file_suffix;

// Program option names and abbreviations.
// Available abbreviations:
// b, d, g, k, n (TODO complete)
static const pair<string, string> rand_seed_opt("random-seed", "r");
static const pair<string, string> algo_opt("algo", "a");
static const pair<string, string> scorer_opt("scorer", "H");
static const pair<string, string> input_data_file_opt("input-file", "i");
static const pair<string, string> target_feature_opt("target-feature", "u");
static const pair<string, string> ignore_feature_opt("ignore-feature", "Y");
static const pair<string, string> force_feature_opt("force-feature", "e");
static const pair<string, string> initial_feature_opt("initial-feature", "f");
static const pair<string, string> output_file_opt("output-file", "o");
static const pair<string, string> log_level_opt("log-level", "l");
static const pair<string, string> log_file_opt("log-file", "F");
static const pair<string, string> log_file_dep_opt_opt("log-file-dep-opt", "L");
static const pair<string, string> target_size_opt("target-size", "C");
static const pair<string, string> threshold_opt("threshold", "T");
static const pair<string, string> jobs_opt("jobs", "j");

// incremental selection flags
static const pair<string, string> inc_target_size_epsilon_opt("inc-target-size-epsilon", "E");
static const pair<string, string> inc_redundant_intensity_opt("inc-redundant-intensity", "D");
static const pair<string, string> inc_interaction_terms_opt("inc-interaction-terms", "U");

// hill-climbing flags
static const pair<string, string> hc_max_evals_opt("max-evals", "m");
static const pair<string, string> hc_max_score_opt("max-score", "A");
static const pair<string, string> hc_fraction_of_remaining_opt("hc-fraction-of-remaining", "O");
static const pair<string, string> hc_cache_size_opt("cache-size", "s");

// mi scorer flags
static const pair<string, string> mi_penalty_opt("mi-penalty", "c");

// Returns a string interpretable by Boost.Program_options
// "name,abbreviation"
string opt_desc_str(const pair<string, string>& opt) {
    string res = string(opt.first);
    if (!opt.second.empty())
        res += string(",") + opt.second;
    return res;
}

/**
 * Display error message about unsupported type and exit
 */
void unsupported_type_exit(const type_tree& tt)
{
    logger().error() << "Type " << tt << "currently not supported.";
    std::cerr << "Error: type " << tt << "currently not supported." << endl;
    exit(1);
}

void unsupported_type_exit(type_node type)
{
    unsupported_type_exit(type_tree(type));
}

int main(int argc, char** argv)
{
    unsigned long rand_seed;
    string log_level;
    string log_file;
    bool log_file_dep_opt;
    feature_selection_parameters fs_params;

    // Declare the supported options.
    options_description desc("Allowed options");

    desc.add_options()
        ("help,h", "Produce help message.\n")
        ("version,v", "Display the version number.\n")

        (opt_desc_str(algo_opt).c_str(),
         value<string>(&fs_params.algorithm)->default_value("simple"),
         string("Feature selection algorithm. Supported algorithms are:\n")
             .append("simple for maximum mutual information,\n")
             .append("smd for stochastic mutual dependency,\n")
             .append("inc for incremental max-relevancy, min-redundancy.\n")
             .append(moses::hc).append(" for moses-hillclimbing,\n")
             .append("random for uniform, random selection.\n")
             .append("The edefault is \"simple\".\n").c_str())

        (opt_desc_str(scorer_opt).c_str(),
         value<string>(&fs_params.scorer)->default_value(mi),
         str(boost::format("Feature selection fitness function (scorer).\n"
                           " Supported scorers are:\n"
                           "%s, for mutual information\n"
                           "%s, for precision (see moses -h for more info)\n")
             % mi % pre).c_str())

        // ======= File I/O opts =========
        (opt_desc_str(input_data_file_opt).c_str(),
         value<string>(&fs_params.input_file),
         "Input table file in DSV format (seperators are comma, "
         "whitespace and tabulation).\n")

        (opt_desc_str(target_feature_opt).c_str(),
         value<string>(&fs_params.target_feature_str),
         "Label of the target feature to fit. If none is given the first one is used.\n")

        ("timestamp-feature",
         value<string>(&fs_params.timestamp_feature_str),
         "Label of the timestamp feature. If none is given it is ignored.\n")

        (opt_desc_str(ignore_feature_opt).c_str(),
         value<vector<string>>(&fs_params.ignore_features_str),
         "Ignore feature from the datasets. Can be used several times "
         "to ignore several features.\n")

        (opt_desc_str(force_feature_opt).c_str(),
         value<vector<string>>(&fs_params.force_features_str),
         "Force feature to be selected. Can be used several times "
         "to force several features. Please note that those features "
         "do not necessarily interact with the selected features, they are "
         "simply added at the end whether or not they are in the selection. "
         "Also ignored features cannot be brought back by forcing them, "
         "that is --ignore-feature has the precedence over --force-feature. \n")

        (opt_desc_str(output_file_opt).c_str(),
         value<string>(&fs_params.output_file),
         "File where to save the results. If empty then it outputs on the stdout.\n")

        (opt_desc_str(log_level_opt).c_str(),
         value<string>(&log_level)->default_value("DEBUG"),
         "Log level; verbosity of logging and debugging messages to "
         "write. Possible levels are NONE, ERROR, WARN, INFO, DEBUG, "
         "FINE. Case does not matter.\n")

        (opt_desc_str(log_file_opt).c_str(),
         value<string>(&log_file)->default_value(default_log_file),
         string("File name where to record the output log.\n")
         .c_str())

        (opt_desc_str(log_file_dep_opt_opt).c_str(),
         string("Use an option-dependent logfile name. The name of "
          "the log is determined by the command-line options; the "
          "base (prefix) of the file is that given by the -F option.  "
          "So, for instance, if the options -L foo -r 123 are given, "
          "then the logfile name will be foo_random-seed_123.log.  "
          "The filename will be truncated to a maximum of ")
          .append(lexical_cast<string>(max_filename_size))
          .append(" characters.\n").c_str())

        // ======= Generic algo opts =========
        (opt_desc_str(initial_feature_opt).c_str(),
         value<vector<string> >(&fs_params.initial_features),
         "Initial feature to search from (not supported by inc). "
         "This option can be used as many times as there are features, "
         "to have them included in the initial feature set. If the "
         "initial feature set is close to the one that maximizes the "
         "quality measure, the selection speed can be greatly increased.\n")

        (opt_desc_str(jobs_opt).c_str(),
         value<unsigned>(&fs_params.jobs)->default_value(1),
         string("Number of threads to use.\n").c_str())

        (opt_desc_str(target_size_opt).c_str(),
         value<unsigned>(&fs_params.target_size)->default_value(0),
         "Feature count.  This option "
         "specifies the number of features to be selected out of "
         "the dataset.  A value of 0 disables this option. \n")

        ("exp-distrib",
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

        (opt_desc_str(threshold_opt).c_str(),
         value<double>(&fs_params.threshold)->default_value(0),
         "Improvment threshold. Floating point number. "
         "Specifies the threshold above which the mutual information "
         "of a feature is considered to be significantly correlated "
         "to the target.  A value of zero means that all features "
         "will be selected. \n"
         "The -C flag over-rides this setting when used with the "
         "-ainc algorithm.\n")

        (opt_desc_str(rand_seed_opt).c_str(),
         value<unsigned long>(&rand_seed)->default_value(1),
         "Random seed.\n")

        ("max-time",
         value<time_t>(&fs_params.max_time)->default_value(INT_MAX),
         "Longest allowed runtime, in seconds (ONLY WORKS FOR hc).\n")

        // ======= Incremental selection params =======
        (opt_desc_str(inc_redundant_intensity_opt).c_str(),
         value<double>(&fs_params.inc_red_intensity)->default_value(0.1),
         "Incremental Selection parameter. Floating-point value must "
         "lie between 0.0 and 1.0.  A value of 0.0 means that no "
         "redundant features will discarded, while 1.0 will cause a "
         "maximal number will be discarded.\n")

        (opt_desc_str(inc_target_size_epsilon_opt).c_str(),
         value<double>(&fs_params.inc_target_size_epsilon)->default_value(0.001),
         "Incremental Selection parameter. Tolerance applied when "
         "selecting for a fixed number of features (option -C).\n")

        (opt_desc_str(inc_interaction_terms_opt).c_str(),
         value<unsigned>(&fs_params.inc_interaction_terms)->default_value(1),
         "Incremental Selection parameter. Maximum number of "
         "interaction terms considered during incremental feature "
         "selection. Higher values make the feature selection more "
         "accurate but is combinatorially more computationally expensive.\n")

        // ======= Hill-climbing only params =======
        (opt_desc_str(hc_max_score_opt).c_str(),
         value<double>(&fs_params.hc_max_score)->default_value(1),
         "Hillclimbing parameter.  The max score to reach, once "
         "reached feature selection halts.\n")

        (opt_desc_str(hc_max_evals_opt).c_str(),
         value<unsigned>(&fs_params.hc_max_evals)->default_value(10000),
         "Hillclimbing parameter.  Maximum number of fitness function "
         "evaluations.\n")

        (opt_desc_str(hc_fraction_of_remaining_opt).c_str(),
         value<double>(&fs_params.hc_fraction_of_remaining)->default_value(0.5),
         "Hillclimbing parameter.  Determine the fraction of the "
         "remaining number of eval to use for the current iteration.\n")

        (opt_desc_str(hc_cache_size_opt).c_str(),
         value<unsigned long>(&fs_params.hc_cache_size)->default_value(1000000),
         "Hillclimbing parameter.  Cache size, so that identical "
         "candidates are not re-evaluated.   Zero means no cache.\n")

        ("hc-crossover",
         value<bool>(&fs_params.hc_crossover)->default_value(false),
         "Hillclimber crossover (see moses --help or man moses for more help)\n")

        ("hc-crossover-pop-size",
         value<unsigned>(&fs_params.hc_crossover_pop_size)->default_value(120),
         "Hillclimber crossover pop size (see moses --help or man moses for more help)\n")

        ("hc-crossover-min-neighbors",
         value<unsigned>(&fs_params.hc_crossover_min_neighbors)->default_value(400),
         "Hillclimber crossover min neighbors (see moses --help or man moses for more help)\n")

        ("hc-widen-search",
         value<bool>(&fs_params.hc_widen_search)->default_value(true),
         "Hillclimber widen_search (see moses --help or man moses for more help)\n")

        // ======= Stochastic max dependency params =======
        ("smd-top-size",
         value<unsigned>(&fs_params.smd_top_size)->default_value(10),
         "Stochastic max dependency parameter. Number of feature subset "
         "candidates to consider building the next superset.\n")

        // ================= mi scoring ====================
        (opt_desc_str(mi_penalty_opt).c_str(),
         value<double>(&fs_params.mi_confi)->default_value(100.0),
         "Mutual-info scoring parameter.  Intensity of the confidence "
         "penalty, in the range (-Inf,+Inf).  100 means no confidence "
         "penalty. This parameter influences how much importance is "
         "attributed to the confidence of the quality measure. The "
         "fewer samples in the data set, the more features the "
         "less confidence in the feature set quality measure.\n")

        
        // ================= pre scoring ====================
        ("pre-penalty",
         value<double>(&fs_params.pre_penalty)->default_value(1.0),
         "Activation penalty (see moses --help or man moses for more info)")

        ("pre-min-activation",
         value<double>(&fs_params.pre_min_activation)->default_value(0.5),
         "Minimum activation (see moses --help or man moses for more info).\n")

        ("pre-max-activation",
         value<double>(&fs_params.pre_max_activation)->default_value(1.0),
         "Maximum activation (see moses --help or man moses for more info).\n")

        ("pre-positive",
         value<bool>(&fs_params.pre_positive)->default_value(true),
         "If 1, then precision, otherwise negative predictive value "
         "(see moses --help or man moses for more info).\n")

        ;

    // XXX TODO add an option to set this ... 
    fs_params.max_time = INT_MAX;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    // Set flags
    log_file_dep_opt = vm.count(log_file_dep_opt_opt.first) > 0;

    // Help
    if (vm.count("help") || argc == 1) {
        cout << desc << std::endl;
        return 1;
    }

    if (vm.count("version")) {
        cout << "feature-selection "
             << opencog::moses::version_string
             << std::endl;
        return 0;
    }

    // Set log
    if (log_file_dep_opt) {
        std::set<std::string> ignore_opt {
            log_file_dep_opt_opt.first,
            log_file_opt.first
        };

        // If the user specified a log file with -F then treat this
        // as the prefix to the long filename.
        string log_file_prefix = default_log_file_prefix;
        if (log_file != default_log_file) {
            log_file_prefix = log_file;
        }

        log_file = determine_log_name(log_file_prefix,
                                      vm, ignore_opt,
                    std::string(".").append(default_log_file_suffix));
    }

    // Remove any existing log files.
    remove(log_file.c_str());
    logger().setFilename(log_file);
    trim(log_level);
    Logger::Level level = logger().getLevelFromString(log_level);
    if (level == Logger::BAD_LEVEL) {
        cerr << "Fatal Error: Log level " << log_level
             << " is incorrect (see --help)." << endl;
        exit(1);
    }
    logger().setLevel(level);
    logger().setBackTraceLevel(Logger::ERROR);

    // Log command-line args
    logger().info() << "feature-selection version "
                    << opencog::moses::version_string;
    string cmdline = "Command line:";
    for (int i = 0; i < argc; ++i) {
         cmdline += " ";
         cmdline += argv[i];
    }
    logger().info(cmdline);

    // init random generator
    randGen().seed(rand_seed);

    // setting OpenMP parameters
    setting_omp(fs_params.jobs);

    // Logger
    logger().info("Read input file %s", fs_params.input_file.c_str());
    // ~Logger

    // fs_params.ignore_features_str = ignore_features_str;

    // Read input_data_file file
    Table table = loadTable(fs_params.input_file,
                            fs_params.target_feature_str,
                            fs_params.timestamp_feature_str,
                            fs_params.ignore_features_str);

    type_tree inferred_tt = table.get_signature();
    type_tree output_tt = get_signature_output(inferred_tt);
    type_node inferred_type = get_type_node(output_tt);

    // Go and do it.
    if ((inferred_type == id::boolean_type) or
        (inferred_type == id::enum_type) or
        (inferred_type == id::contin_type)) {
        feature_selection(table, fs_params);
    } else {
        unsupported_type_exit(inferred_type);
    }
}
