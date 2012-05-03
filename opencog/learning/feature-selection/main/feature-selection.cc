/** feature-selection.cc ---
 *
 * Copyright (C) 2011 OpenCog Foundation
 *
 * Author: Nil Geisweiller <nilg@desktop>
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

#include <iostream>
#include <fstream>
#include <memory>
#include <stdio.h>

#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/Logger.h>
#include <opencog/util/lru_cache.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/iostreamContainer.h>
#include <opencog/util/log_prog_name.h>

#include <opencog/comboreduct/combo/table.h>

#include <opencog/learning/moses/optimization/optimization.h>

#include "feature-selection.h"
#include "../feature_optimization.h"
#include "../feature_scorer.h"

using namespace std;
using namespace opencog;
using namespace combo;

using namespace boost::program_options;
using boost::lexical_cast;
using boost::trim;

// Assorted defaults.
const static unsigned max_filename_size = 255;

static const string default_log_file_prefix = "feature-selection";
static const string default_log_file_suffix = "log";
static const string default_log_file = default_log_file_prefix + "." + default_log_file_suffix;

// Program option names and abbreviations.
static const pair<string, string> rand_seed_opt("random-seed", "r");
static const pair<string, string> algo_opt("algo", "a");
static const pair<string, string> input_data_file_opt("input-file", "i");
static const pair<string, string> target_feature_opt("target-feature", "u");
static const pair<string, string> initial_feature_opt("initial-feature", "f");
static const pair<string, string> max_evals_opt("max-evals", "m");
static const pair<string, string> output_file_opt("output-file", "o");
static const pair<string, string> log_level_opt("log-level", "l");
static const pair<string, string> log_file_opt("log-file", "F");
static const pair<string, string> log_file_dep_opt_opt("log-file-dep-opt", "L");
static const pair<string, string> cache_size_opt("cache-size", "s");
static const pair<string, string> target_size_opt("target-size", "C");
static const pair<string, string> threshold_opt("threshold", "T");
static const pair<string, string> confidence_penalty_intensity_opt("confidence-penalty-intensity", "c");
static const pair<string, string> max_score_opt("max-score", "A");
static const pair<string, string> jobs_opt("jobs", "j");
static const pair<string, string> inc_target_size_epsilon_opt("inc-target-size-epsilon", "E");
static const pair<string, string> inc_redundant_intensity_opt("inc-redundant-intensity", "D");
static const pair<string, string> inc_interaction_terms_opt("inc-interaction-terms", "U");
static const pair<string, string> hc_fraction_of_remaining_opt("hc-fraction-of-remaining", "O");

string opt_desc_str(const pair<string, string>& opt) {
    return string(opt.first).append(",").append(opt.second);
}

/**
 * Display error message about unsupported type and exit
 */
void unsupported_type_exit(const type_tree& tt)
{
    std::cerr << "error: type " << tt << "currently not supported" << std::endl;
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

        (opt_desc_str(rand_seed_opt).c_str(),
         value<unsigned long>(&rand_seed)->default_value(1),
         "Random seed.\n")

        (opt_desc_str(algo_opt).c_str(),
         value<string>(&fs_params.algorithm)->default_value(mmi),
         string("Feature selection algorithm. Supported algorithms are:\n")
             /*
              * We're not going to support univariate or sa any time
              * soon, and maybe never; they're kind-of deprecated in
              * MOSES, at the moment.
             .append(un).append(" for univariate,\n")
             .append(sa).append(" for simulated annealing,\n")
             */
             .append(mmi).append(" for maximal mutual information,\n")
             .append(hc).append(" for hillclimbing (unsupported),\n")
             .append(inc).append(" for incremental mutual information.\n").c_str())

        (opt_desc_str(input_data_file_opt).c_str(),
         value<string>(&fs_params.input_file),
         "Input table file in DSV format (seperators are comma, whitespace and tabulation).\n")

        (opt_desc_str(target_feature_opt).c_str(),
         value<string>(&fs_params.target_feature),
         "Label of the target feature to fit. If none is given the first one is used.\n")

        (opt_desc_str(output_file_opt).c_str(),
         value<string>(&fs_params.output_file),
         "File where to save the results. If empty then it outputs on the stdout.\n")

        (opt_desc_str(jobs_opt).c_str(),
         value<unsigned>(&fs_params.jobs)->default_value(1),
         string("Number of threads to use.\n").c_str())

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


        (opt_desc_str(target_size_opt).c_str(),
         value<unsigned>(&fs_params.target_size)->default_value(0),
            "Feature count.  This option "
            "specifies the number of features to be selected out of "
            "the dataset.  A value of 0 disables this option. \n")

        (opt_desc_str(threshold_opt).c_str(),
         value<double>(&fs_params.threshold)->default_value(0),
            "Improvment threshold. Floating point number. "
            "Specifies the threshold above which the mutual information "
            "of a feature is considered to be significantly correlated "
            "to the target.  A value of zero means that all features "
            "will be selected. \n"
            "For the -ainc algo only, the -C flag over-rides this setting.\n")

// XXX XXX XXX XXXXXXXXXXXXXXXXXXX

        (opt_desc_str(initial_feature_opt).c_str(), value<vector<string> >(&fs_params.initial_features),
         "Initial feature to search from. This option can be used as many times as features to include in the initial feature set. An initial feature set close to the one maximizing the feature quality measure can greatly increase feature selection speed.\n")

        (opt_desc_str(max_evals_opt).c_str(),
         value<unsigned>(&fs_params.max_evals)->default_value(10000),
         "Maximum number of fitness function evaluations.\n")

        (opt_desc_str(cache_size_opt).c_str(),
         value<unsigned long>(&fs_params.cache_size)->default_value(1000000),
         "Cache size, so that identical candidates are not re-evaluated, 0 means no cache.\n")

        (opt_desc_str(confidence_penalty_intensity_opt).c_str(),
         value<double>(&fs_params.confi)->default_value(1.0),
         "Intensity of the confidence penalty, in [0,+Inf), 0 means no confidence penalty. This parameter influences how much importance we attribute to the confidence of the feature quality measure. The less samples in the data set, the more features the less confidence in the feature set quality measure.\n")

        (opt_desc_str(inc_redundant_intensity_opt).c_str(),
         value<double>(&fs_params.inc_red_intensity)->default_value(0.1),
         "Incremental Selection parameter. Floating-point value must "
         "lie between 0.0 and 1.0.  A value of 0.0 means that no "
         "redundant features will discarded, while 1.0 will cause a "
         "maximal number will be discarded.\n")

        (opt_desc_str(inc_target_size_epsilon_opt).c_str(),
         value<double>(&fs_params.inc_target_size_epsilon)->default_value(0.001),
         "Incremental Selection parameter. Toleance applied when "
         "selecting for a fixed number of features (option -C).\n")

        (opt_desc_str(inc_interaction_terms_opt).c_str(),
         value<unsigned>(&fs_params.inc_interaction_terms)->default_value(1),
         "Incremental Selection parameter. Maximum number of "
         "interaction terms considered during incremental feature "
         "selection. Higher values make the feature selection more "
         "accurate but is exponentially more computationally expensive.\n")

        (opt_desc_str(max_score_opt).c_str(),
         value<double>(&fs_params.max_score)->default_value(1),
         "Hillclimbing parameter. The max score to reach, once "
         "reached feature selection halts.\n")

        (opt_desc_str(hc_fraction_of_remaining_opt).c_str(),
         value<double>(&fs_params.hc_fraction_of_remaining)->default_value(0.1),
         "Hillclimbing parameter. Determine the fraction of the "
         "remaining number of eval to use for the current iteration.\n")

        ;

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

    type_tree inferred_tt = infer_data_type_tree(fs_params.input_file);
    type_tree output_tt = type_tree_output_type_tree(inferred_tt);
    type_node inferred_type = *output_tt.begin();

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

    // Find the position of the target feature (the first one by default)
    int target_pos = fs_params.target_feature.empty()? 0
        : findTargetFeaturePosition(fs_params.input_file,
                                    fs_params.target_feature);
    // Read input_data_file file
    Table table = istreamTable(fs_params.input_file, target_pos);

    // Go and do it.
    if(inferred_type == id::boolean_type) {
        feature_selection(table, fs_params);
    } else {
        unsupported_type_exit(inferred_type);
    }
}
