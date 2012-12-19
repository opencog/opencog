/** eval-features.cc --- 
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

#include <boost/program_options.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <opencog/util/iostreamContainer.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/util/numeric.h>

#include <opencog/comboreduct/table/table_io.h>

#include "eval-features.h"

using namespace boost::program_options;
using boost::lexical_cast;

/**
 * Program to output the feature quality of a feature set given a data
 * set. One can provide a list of feature sets instead of one feature
 * set and output the results for each data set.
 */

int main(int argc, char** argv)
{
    // program options, see options_description below for their meaning
    eval_features_parameters pa;
    unsigned long rand_seed;
    string target_feature_str;
    vector<string> ignore_features_str;

    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Produce help message.\n")

        (opt_desc_str(rand_seed_opt).c_str(),
         value<unsigned long>(&rand_seed)->default_value(1),
         "Random seed.\n")
        
        (opt_desc_str(scorer_opt).c_str(),
         value<string>(&pa.scorer)->default_value(mi),
         str(boost::format("Feature set scorer.\n"
                           " Supported scorers are:\n"
                           "%s, for mutual information\n"
                           "%s, for precision (see moses -h for more info)\n")
             % mi % pre).c_str())
        
        (opt_desc_str(input_file_opt).c_str(),
         value<string>(&pa.input_file),
         "Input table file.\n")
                
        (opt_desc_str(target_feature_opt).c_str(),
         value<string>(&target_feature_str),
         "Label of the target feature to fit. If none is given the first one is used.\n")

        (opt_desc_str(ignore_feature_opt).c_str(),
         value<vector<string>>(&ignore_features_str),
         "Ignore feature from the datasets. Can be used several times "
         "to ignore several features.\n")

        (opt_desc_str(output_file_opt).c_str(), value<string>(&pa.output_file),
         "File where to save the results. If empty then it outputs on the stdout.\n")
        
        (opt_desc_str(feature_opt).c_str(), value<vector<string> >(&pa.features),
         "Feature to consider. Can be used several time for several features.\n")
        
        (opt_desc_str(features_file_opt).c_str(), value<string>(&pa.features_file),
         "File containing feature sets to consider. Each feature set per line, with features seperated by comma. The results of each feature set is displayed in a row seperated by a whitespace. So if there several combo programs (if any at all), each row corresponds to a program and each column corresponds to a feature set.\n")
        
        (opt_desc_str(confidence_penalty_intensity_opt).c_str(),
         value<double>(&pa.confidence_penalty_intensity)->default_value(0),
         "Confidence penalty of the feature set (the larger the more penalized). Value between 0 to inf. This penalty is different than the complexity penalty as it takes into account the sample number size. The larger the sample size the more confident.\n")
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if(vm.count("help") || argc == 1) {
        cout << desc << "\n";
        return 1;
    }

    // init random generator
    randGen().seed(rand_seed);
    read_eval_output_results(pa);
}
