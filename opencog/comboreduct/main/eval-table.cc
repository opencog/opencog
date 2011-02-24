/** eval-table.cc --- 
 *
 * Copyright (C) 2011 Nil Geisweiller
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

#include <opencog/comboreduct/combo/table.h>

//ant_combo_vocabulary is used only for the boolean core vocabulary
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>

using namespace boost::program_options;
using boost::lexical_cast;
using namespace ant_combo;
using namespace opencog;

/**
 * Convert a string representing a combo program in a combo_tree.
 *
 * @param combo_prog_str   the string containing the combo program
 * @param has_labels       true if the combo program has labels instead of
 * place holders (like and(#start #up) instead of and(#23 #134)
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

/**
 * Program to evaluate a combo program over a data set repsented as csv file.
 */
int main(int argc,char** argv) { 

    // program options, see options_description below for their meaning
    unsigned long rand_seed;
    string input_table_file;
    string combo_program_str;
    bool labels;
    // bool residual_error;

    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Produce help message.\n")
        ("rand-seed,r", value<unsigned long>(&rand_seed)->default_value(1),
         "Random seed.\n")
        ("input-table,i", value<string>(&input_table_file), "Input table file.\n")
        ("combo-program,c", value<string>(&combo_program_str),
         "Combo program to evaluate against the input table.\n")
        ("labels,l", "If enabled then the combo program is expected to contain variables labels #labels1, etc, instead of place holders. For instance one provide the combo program \"and(#large #tall)\" instead of \"and(#24 #124)\". In such a case it is expected that the input data file contains the labels as first row.\n")
        // ("residual-error,e", "If enabled then residual error of the combo program against the output of the data set is printed.\n")
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if(vm.count("help") || argc == 1) {
        cout << desc << "\n";
        return 1;
    }

    // set variables
    labels = vm.count("labels");
    // residual_error = vm.count("residual-error");

    // init random generator
    opencog::MT19937RandGen rng(rand_seed);

    // read input_table_file file
    type_node data_type = inferDataType(input_table_file);

    if(data_type == id::boolean_type) {
        truth_table_inputs it;
        partial_truth_table ct;
        istreamTable<truth_table_inputs,
                     partial_truth_table, bool>(input_table_file, it, ct);

        // read combo program
        combo_tree tr = str2combo_tree_label(combo_program_str,
                                             labels, it.get_labels());

        // evaluated tr over input table
        partial_truth_table ct_tr = partial_truth_table(tr, it, rng);
        ct_tr.set_label(ct.get_label());

        // print target variable result
        std::cout << ct_tr << std::endl;;

        // print residual error
        // if(residual_error)
        //     cout << "Residual error = " <<
        //         ct.root_mean_square_error(ct_tr) << endl;
    } else if(data_type == id::contin_type) {
        contin_input_table it;
        contin_table ct;
        istreamTable<contin_input_table,
                     contin_table, contin_t>(input_table_file, it, ct);

        // read combo program
        combo_tree tr = str2combo_tree_label(combo_program_str,
                                             labels, it.get_labels());

        // evaluated tr over input table
        contin_table ct_tr = contin_table(tr, it, rng);
        ct_tr.set_label(ct.get_label());

        // print target variable result
        std::cout << ct_tr << std::endl;;

        // print residual error
        // if(residual_error)
        //     cout << "Residual error = " <<
        //         ct.root_mean_square_error(ct_tr) << endl;
    }

}


