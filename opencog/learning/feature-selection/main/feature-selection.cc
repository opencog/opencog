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
#include <boost/assign/list_of.hpp>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/Logger.h>
#include <opencog/util/lru_cache.h>
#include <opencog/util/algorithm.h>
#include <opencog/util/iostreamContainer.h>
#include <opencog/util/log_prog_name.h>

#include <opencog/comboreduct/combo/table.h>

#include <opencog/learning/moses/moses/optimization.h>

#include "feature-selection.h"
#include "../feature_optimization.h"
#include "../feature_scorer.h"

using namespace boost::program_options;
using boost::lexical_cast;
using boost::assign::list_of;
using namespace std;
using namespace opencog;
using namespace combo;

const static unsigned max_filename_size = 255;

/**
 * Display error message about unsupported type and exit
 */
void unsupported_type_exit(const type_tree& tt) {
    std::cerr << "error: type " << tt << "currently not supported" << std::endl;
    exit(1);
}
void unsupported_type_exit(type_node type) {
    unsupported_type_exit(type_tree(type));
}

int main(int argc, char** argv) {

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
         value<string>(&fs_params.algorithm)->default_value(hc),
         string("Feature selection algorithm. Supported algorithms are ").append(un).append(" for univariate, ").append(sa).append(" for simulated annealing, ").append(hc).append(" for hillclimbing, ").append(inc).append(" for incremental.").c_str())
        (opt_desc_str(input_data_file_opt).c_str(),
         value<string>(&fs_params.input_file),
         "Input table file, DSV file using comman, whitespace or tabulation as seperator.\n")
        (opt_desc_str(output_file_opt).c_str(),
         value<string>(&fs_params.output_file),
         "File where to save the results. If empty then it outputs on the stdout.\n")
        (opt_desc_str(initial_feature_opt).c_str(), value<vector<string> >(&fs_params.initial_features),
         "Initial feature to search from. This option can be used as many times as features to include in the initial feature set. An initial feature set close to the one maximizing the feature quality measure can greatly increase feature selection speed.\n")
        (opt_desc_str(max_evals_opt).c_str(),
         value<unsigned>(&fs_params.max_evals)->default_value(10000),
         "Maximum number of fitness function evaluations.\n")
        (opt_desc_str(log_level_opt).c_str(),
         value<string>(&log_level)->default_value("DEBUG"),
         "Log level, possible levels are NONE, ERROR, WARN, INFO, DEBUG, FINE. Case does not matter.\n")
        (opt_desc_str(log_file_dep_opt_opt).c_str(),
         string("The name of the log is determined by the options, for instance if feature-selection is called with -r 123 the log name is feature-selection_random-seed_123.log. Note that the name will be truncated in order not to be longer than ").append(lexical_cast<string>(max_filename_size)).append(" characters.\n").c_str())
        (opt_desc_str(log_file_opt).c_str(),
         value<string>(&log_file)->default_value(default_log_file),
         string("File name where to write the log. This option is overwritten by ").append(log_file_dep_opt_opt.first).append(".\n").c_str())
        (opt_desc_str(cache_size_opt).c_str(),
         value<unsigned long>(&fs_params.cache_size)->default_value(1000000),
         "Cache size, so that identical candidates are not re-evaluated, 0 means no cache.\n")
        (opt_desc_str(complexity_penalty_intensity_opt).c_str(),
         value<double>(&fs_params.cpi)->default_value(0.0),
         "Intensity of the feature complexity penalty, in [0,+Inf), 0 means no complexity penalty.\n")
        (opt_desc_str(confidence_penalty_intensity_opt).c_str(),
         value<double>(&fs_params.confi)->default_value(1.0),
         "Intensity of the confidence penalty, in [0,+Inf), 0 means no confidence penalty. This parameter influences how much importance we attribute to the confidence of the feature quality measure. The less samples in the data set, the more features the less confidence in the feature set quality measure.\n")
        (opt_desc_str(resources_opt).c_str(),
         value<double>(&fs_params.resources)->default_value(10000),
         "Resources allocated to the learning algorithm that take in input the selected features. More resource means that the feature set can be larger (as long as it has enough confidence). In this case the algo is supposed to be MOSES and the resources is the number of evaluations.\n")
        (opt_desc_str(max_score_opt).c_str(),
         value<double>(&fs_params.max_score)->default_value(1),
         "For MOSES based algorithms. The max score to reach, once reached feature selection halts.\n")
        (opt_desc_str(fraction_of_remaining_opt).c_str(),
         value<unsigned>(&fs_params.hc_fraction_of_remaining)->default_value(10),
         "Hillclimbing parameter. Determine the fraction of the remaining number of eval to use for the current iteration.\n")
        (opt_desc_str(feature_selection_intensity_opt).c_str(),
         value<double>(&fs_params.inc_intensity)->default_value(0),
         "Incremental Selection parameter. Value between 0 and 1. 0 means all features are selected, 1 corresponds to the stronger selection pressure, probably no features are selected at 1.\n")
        (opt_desc_str(feature_selection_target_size_opt).c_str(),
         value<unsigned>(&fs_params.inc_target_size)->default_value(0),
         "Incremental Selection parameter. The number of features to attempt to select. This option overwrites feature-selection-intensity. 0 means disabled.\n")
        (opt_desc_str(redundant_feature_intensity_opt).c_str(),
         value<double>(&fs_params.inc_rintensity)->default_value(0.1),
         "Incremental Selection parameter. Value between 0 and 1. 0 means no redundant features are discarded, 1 means redudant features are maximally discarded. This option is only active when feature selection is active.\n")
        (opt_desc_str(feature_selection_interaction_terms_opt).c_str(),
         value<unsigned>(&fs_params.inc_interaction_terms)->default_value(1),
         "Incremental Selection parameter. Maximum number of interaction terms considered during feature selection. Higher values make the feature selection more accurate but is computationally expensive.\n")
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    // set flags
    log_file_dep_opt = vm.count(log_file_dep_opt_opt.first) > 0;

    // help
    if (vm.count("help") || argc == 1) {
        cout << desc << std::endl;
        return 1;
    }

    // set log
    if(log_file_dep_opt) {
        std::set<std::string> ignore_opt = list_of(log_file_dep_opt_opt.first);
        log_file = determine_log_name(default_log_file_prefix,
                                      vm, ignore_opt,
                                      std::string(".").append(default_log_file_suffix));
    }

    type_node inferred_type = inferDataType(fs_params.input_file);

    // remove log_file
    remove(log_file.c_str());
    logger().setFilename(log_file);
    logger().setLevel(logger().getLevelFromString(log_level));
    logger().setBackTraceLevel(Logger::ERROR);

    // init random generator
    MT19937RandGen rng(rand_seed);

    auto_ptr<ifstream> in(open_data_file(fs_params.input_file));

    if(inferred_type == id::boolean_type) {
        // read input_data_file file
        truth_table table(*in);
        in->close();
        ctruth_table ctable = table.compress();
        feature_selection(table, fs_params, rng);
    } else {
        unsupported_type_exit(inferred_type);
    }
}
