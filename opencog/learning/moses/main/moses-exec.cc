/** moses-exec.cc --- 
 *
 * Copyright (C) 2010 Nil Geisweiller
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
#include <iostream>
#include <fstream>
#include <stdio.h>

#include <boost/program_options.hpp>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/Logger.h>

#include <opencog/comboreduct/combo/eval.h>
#include <opencog/comboreduct/combo/table.h>

#include "../moses/moses.h"
#include "../moses/optimization.h"
#include "../moses/scoring_functions.h"
#include "../moses/scoring.h"

using namespace boost::program_options;
using namespace std;
using namespace moses;
using namespace reduct;
using opencog::logger;

int main(int argc,char** argv) { 

    unsigned long rand_seed;
    string input_table_file;
    unsigned long max_evals;
    unsigned int max_gens;
    string log_level;
    string log_file;
    
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("random-seed,r", value<unsigned long>(&rand_seed)->default_value(1),
         "random seed")
        ("max-evals,m", value<unsigned long>(&max_evals)->default_value(10000),
         "maximum number of fitness function evaluations")
        ("max-gens,m", value<unsigned int>(&max_gens)->default_value(1000),
         "maximum number of demes to generate")
        ("input-file,i", value<string>(&input_table_file),
         "input table file")
        ("log-level,l", value<string>(&log_level)->default_value("DEBUG"),
         "log level, possible levels are NONE, ERROR, WARN, INFO, DEBUG, FINE. Case does not matter")
        ("log-file,f", value<string>(&log_file)->default_value("moses.log"),
         "file name where to write the log")
        ;
    
    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);    
    
    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    // set log
    // remove log_file
    remove(log_file.c_str());
    logger().setFilename(log_file);
    logger().setLevel(logger().getLevelFromString(log_level));
    
    // if (vm.count("input-file")) {
    //     cout << "Compression level was set to " 
    //          << vm["compression"].as<int>() << ".\n";
    // } else {
    //     cout << "Compression level was not set.\n";
    // }


    // init random generator
    opencog::MT19937RandGen rng(rand_seed);

    ifstream in(input_table_file.c_str());
    contin_table contintable;
    RndNumTable inputtable;
    contin_vector input_vec;
    contin_t input;
    char check;
    while (!in.eof()) {
        in>>input;
        check = in.get();
        if (check == '\n') {
            contintable.push_back(input);
            inputtable.push_back(input_vec);
            input_vec.clear();
        }
        else {
            input_vec.push_back(input);
        }
    }
    unsigned int arity = inputtable[0].size();

    type_tree tt(id::lambda_type);
    tt.append_children(tt.begin(), id::contin_type, arity + 1);

    contin_score_sqr score(contintable, inputtable, rng);
    contin_bscore bscore(contintable, inputtable, rng);

    metapopulation<contin_score_sqr, contin_bscore, univariate_optimization> 
    metapop(rng,
            combo_tree(id::plus),
            tt,contin_reduction(rng),
            score,
            bscore,
            univariate_optimization(rng));
    
    moses::moses(metapop, max_evals, 0);
}
