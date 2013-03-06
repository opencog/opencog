/** gen-table.cc --- 
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

#include "gen-table.h"

#include <iostream>
#include <fstream>
#include <string>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/random.h>
#include <opencog/util/log_prog_name.h>

#include <opencog/comboreduct/interpreter/eval.h>
#include <opencog/comboreduct/table/table.h>
#include <opencog/comboreduct/table/table_io.h>
#include <opencog/comboreduct/type_checker/type_tree.h>

using namespace boost::program_options;
using namespace opencog;
using namespace combo;
using namespace std;

string opt_desc_str(const pair<string, string>& opt) {
    return string(opt.first).append(",").append(opt.second);
}

// combo_tree
combo_tree str_to_combo_tree(const string& combo_str) {
    stringstream ss;
    combo_tree tr;
    ss << combo_str;
    ss >> tr;
    return tr;
}

int main(int argc, char** argv) {

    // program options, see options_description below for their meaning
    unsigned long rand_seed;
    bool header;
    string combo_str;
    string combo_file_str;
    int nsamples;
    float min_contin;
    float max_contin;
    string output_file;
    int target_index;
    
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Produce help message.\n")
        (opt_desc_str(rand_seed_opt).c_str(),
         value<unsigned long>(&rand_seed)->default_value(1),
         "Random seed.\n")
        (opt_desc_str(header_opt).c_str(),
         value<bool>(&header)->default_value(true),
         "If 1, the first row is the header of the DSV file (seperators are comma, whitespace and tabulation).\n")
        (opt_desc_str(combo_program_opt).c_str(),
         value<string>(&combo_str),
         "Combo program to generate the output (wrap it between single quotes "
         "so that bash doesn't perform variable substitution).\n")
        (opt_desc_str(combo_program_file_opt).c_str(),
         value<string>(&combo_file_str),
         "Name of a file containing a combo program to generate the output. "
         "It has the precedence over option -y.\n")
        (opt_desc_str(nsamples_opt).c_str(),
         value<int>(&nsamples)->default_value(-1),
         "Size (number of rows) of the table. If negative then the number is determined automatically (a complete truth table is generated if the combo program has boolean inputs).\n")
        (opt_desc_str(min_contin_opt).c_str(),
         value<float>(&min_contin)->default_value(0),
         "Min of an input value chosen randomly, only used when the problem takes continuous inputs.\n")
        (opt_desc_str(max_contin_opt).c_str(),
         value<float>(&max_contin)->default_value(1),
         "Max of an input value chosen randomly, only used when the problem takes continuous inputs.\n")
        (opt_desc_str(target_index_opt).c_str(),
         value<int>(&target_index)->default_value(0),
         "Index of the output column.\n")
        (opt_desc_str(output_file_opt).c_str(),
         value<string>(&output_file)->default_value(""),
         "File where to save the table. If empty then it outputs on the stdout.\n");

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help") || argc == 1) {
        cout << desc << "\n";
        return 1;
    }

    // init random generator
    randGen().seed(rand_seed);

    /// @todo translate in case the variables are names rather than
    /// place holders
    if (!combo_file_str.empty()) {
        ifstream combo_file(combo_file_str);
        getline(combo_file, combo_str);
    }
    combo_tree tr = str_to_combo_tree(combo_str);
    type_tree tt = infer_type_tree(tr);
    OC_ASSERT(is_well_formed(tt));

    // generate input table
    Table table(tr, nsamples, min_contin, max_contin);

    // output the table
    if(output_file.empty())
        ostreamTable(cout, table);
    else {
        ofstream of(output_file.c_str());
        ostreamTable(of, table);
        of.close();
    }
}

