/** eval-diversity.cc --- 
 *
 * Copyright (C) 2012 OpenCog Foundation
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

/**
 * Tool that loads several moses outputs or tables and compute the
 * diversity of the pool of candidates.
 */

#include <boost/program_options.hpp>

#include <opencog/util/oc_assert.h>

using namespace boost::program_options;
using namespace std;
// using namespace opencog;

struct eval_diversity_params {
    vector<string> moses_files;
    float diversity_p_norm;
};

int main(int argc, char** argv) {
    eval_diversity_params edp;
    
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "Produce help message.\n")

        ("moses-file,m", value<vector<string>>(&edp.moses_files),
         "File containing the candidates as output by moses. The output "
         "must contain the bscores (see option -t or moses). "
         "Can be used several times for several files")

        ("diversity-p-norm",
         value<float>(&edp.diversity_p_norm)->default_value(2.0),
         "Set the parameter of the p-norm used to compute the distance between "
         "behavioral scores used for the diversity penalty. A value of 1.0 "
         "correspond to the Manhatan distance. A value of 2.0 corresponds to "
         "the Euclidean distance. A value of 0.0 or less correspond to the "
         "max component-wise. Any other value corresponds to the general case.\n")

        ;

        variables_map vm;
    try {
        store(parse_command_line(argc, argv, desc), vm);
    }
    catch (error& e) {
        OC_ASSERT(false, "Fatal error: invalid or duplicated argument:\n\t%s\n",
                  e.what());
    }
    notify(vm);

    if (vm.count("help") || argc == 1) {
        cout << desc << endl;
        return 1;
    }

    // load the bscores
    // TODO
    // vector<behavioral_score> bscores;
    // foreach(string file, edp.moses_files) {
        
    // }
}
