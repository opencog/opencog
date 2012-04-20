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

#include <opencog/util/iostreamContainer.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/util/numeric.h>

#include "eval-features.h"

using namespace boost::program_options;
using boost::lexical_cast;

/**
 * Program to output the feature quality of a feature set given a data
 * set. Optionally the variable correpsonding to the output of the
 * data set (the last column) can be substituted by a combo program
 * (or several combo programs, one for each results). Likewise one can
 * provide a list of feature sets instead of one feature set and
 * output the results for each data set.
 */

/**
 * Convert a string representing a combo program in a combo_tree.
 *
 * @param combo_prog_str   the string containing the combo program
 * @param has_labels       true if the combo program has labels instead of
 * place holders (like and($start $up) instead of and($23 $134)
 * @param labels           a vector of labels
 * @return                 the combo_tree
 */
combo_tree str2combo_tree_label(const std::string& combo_prog_str,
                                bool has_labels,
                                const std::vector<std::string>& labels) {
    // combo pogram with place holders
    std::string combo_prog_ph_str = 
        has_labels? l2ph(combo_prog_str, labels) : combo_prog_str;
    std::stringstream ss(combo_prog_ph_str);
    combo_tree tr;
    ss >> tr;
    return tr;
}

int main(int argc,char** argv) { 

    // program options, see options_description below for their meaning
    eval_features_parameters pa;
    unsigned long rand_seed;

    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Produce help message.\n")
        (opt_desc_str(rand_seed_opt).c_str(),
         value<unsigned long>(&rand_seed)->default_value(1),
         "Random seed.\n")
        (opt_desc_str(input_data_file_opt).c_str(),
         value<string>(&pa.input_table_file),
         "Input table file.\n")
        (opt_desc_str(combo_str_opt).c_str(),
         value<vector<string> >(&pa.combo_programs),
         "Combo program to replace the output (last column) of the data set. It can be used several time so that several programs are evaluated at once.\n")
        (opt_desc_str(combo_prog_file_opt).c_str(),
         value<string>(&pa.combo_programs_file),
         "File containing combo programs to replace the output (last column) of the data set. Each combo program in the file is seperated by a new line. For each program, all features sets are evaluated and their results displayed.\n")
        (opt_desc_str(labels_opt).c_str(), "If enabled then the combo program is expected to contain variables labels $labels1, etc, instead of place holders. For instance one provide the combo program \"and($large $tall)\" instead of \"and($24 $124)\". In such a case it is expected that the input data file contains the labels as first row.\n")
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

    // set variables
    pa.has_labels = vm.count("labels");

    // init random generator
    randGen().seed(rand_seed);

    // read input_table_file file
    type_tree data_tt = infer_data_type_tree(pa.input_table_file);
    type_tree output_tt = type_tree_output_type_tree(data_tt);
    type_node data_type = *output_tt.begin();

    if(data_type == id::boolean_type) {
        read_eval_output_results(pa);
    } else if(data_type == id::contin_type) {
        read_eval_output_results(pa);
    }

}
