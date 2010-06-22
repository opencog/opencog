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

// for operator>> to combo 
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h> 

#include "../moses/moses.h"
#include "../moses/optimization.h"
#include "../moses/scoring_functions.h"
#include "../moses/scoring.h"

using namespace boost::program_options;
using namespace std;
using namespace moses;
using namespace reduct;
using opencog::logger;
using namespace ant_combo;

int main(int argc,char** argv) { 

    // program options, see options_description below for their meaning
    unsigned long rand_seed;
    string input_table_file;
    unsigned long max_evals;
    long result_count;
    int max_gens;
    string log_level;
    string log_file;
    float variance;
    vector<string> ignore_ops_str;
    string opt_algo; //optimization algorithm
    static const string un="un"; // univariate
    static const string sa="sa"; // simulation annealing
    static const string hc="hc"; // hillclimbing
    string exemplar_str;
    
    // Declare the supported options.
    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("random-seed,r", value<unsigned long>(&rand_seed)->default_value(1),
         "random seed")
        ("max-evals,m", value<unsigned long>(&max_evals)->default_value(10000),
         "maximum number of fitness function evaluations")
        ("result-count,r", value<long>(&result_count)->default_value(10),
         "the number of non-dominated best results to return ordered according to their score, if negative then returns all of them")
        ("max-gens,g", value<int>(&max_gens)->default_value(-1),
         "maximum number of demes to generate and optimize, negative means no generation limit")
        ("input-file,i", value<string>(&input_table_file),
         "input table file")
        ("log-level,l", value<string>(&log_level)->default_value("DEBUG"),
         "log level, possible levels are NONE, ERROR, WARN, INFO, DEBUG, FINE. Case does not matter")
        ("log-file,f", value<string>(&log_file)->default_value("moses.log"),
         "file name where to write the log")
        ("variance,v", value<float>(&variance)->default_value(0),
         "variance of an assumed Gaussian around each candidate's output, useful if the data are noisy or to control an Occam's razor bias, 0 or negative means no Occam's razor, otherwise the higher v the stronger the Occam's razor")
        ("ignore-operator,n", value<vector<string> >(&ignore_ops_str),
         "ignore the following operator in the program solution, can be used several times, for moment only div, sin, exp and log can be ignored")
        ("opt-alg,a", value<string>(&opt_algo)->default_value(un),
         "optimization algorithm, current supported algorithms are univariate (un), simulation annealing (sa), hillclimbing (hc)")
        ("exemplar,e", value<string>(&exemplar_str)->default_value("+"),
         "start the search with a given exemplar")
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);    
    
    if (vm.count("help") || argc == 1) {
        cout << desc << "\n";
        return 1;
    }

    // set log
    // remove log_file
    remove(log_file.c_str());
    logger().setFilename(log_file);
    logger().setLevel(logger().getLevelFromString(log_level));
    logger().setBackTraceLevel(opencog::Logger::ERROR);
    // init random generator
    opencog::MT19937RandGen rng(rand_seed);

    // read the input_table_file file
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

    // convert ignore_ops_str to the set of actual operators to ignore
    vertex_set ignore_ops;
    foreach(const string& s, ignore_ops_str) {
        vertex v;
        if(builtin_str_to_vertex(s, v))
            ignore_ops.insert(v);
        else {
            std::cerr << "error: " << s 
                      << " is not recognized as combo operator" << std::endl;
            return 1;
        }
    }

    // set the initial exemplar
    combo_tree exemplar;
    stringstream ss;
    ss << exemplar_str;
    ss >> exemplar;

    // set alphabet size
    int alphabet_size = 8 - ignore_ops.size(); // 8 is roughly the
                                               // number of operators
                                               // in contin formula,
                                               // it will have to be
                                               // adapted

    occam_contin_bscore bscore(contintable, inputtable,
                               variance, alphabet_size, rng);
    bscore_based_score<occam_contin_bscore> score(bscore);

    if(opt_algo == un) { // univariate
        metapopulation<bscore_based_score<occam_contin_bscore>,
                       occam_contin_bscore, 
                       univariate_optimization> 
            metapop(rng, exemplar, tt,
                    contin_reduction(rng), score, bscore,
                    univariate_optimization(rng));
        moses::moses(metapop, max_evals, max_gens, 0, ignore_ops);
        metapop.print_best(result_count);
    } else if(opt_algo == sa) { // simulation annealing
        metapopulation<bscore_based_score<occam_contin_bscore>,
                       occam_contin_bscore,
                       simulated_annealing> 
            metapop(rng, exemplar, tt,
                    contin_reduction(rng), score, bscore,
                    simulated_annealing(rng));
        moses::moses(metapop, max_evals, max_gens, 0, ignore_ops);
        metapop.print_best(result_count);
    } else if(opt_algo == hc) { // hillclimbing
        metapopulation<bscore_based_score<occam_contin_bscore>,
                       occam_contin_bscore,
                       iterative_hillclimbing> 
            metapop(rng, exemplar, tt,
                    contin_reduction(rng), score, bscore,
                    iterative_hillclimbing(rng));
        moses::moses(metapop, max_evals, max_gens, 0, ignore_ops);
        metapop.print_best(result_count);
    } else {
        std::cerr << "Unknown optimization algo " << opt_algo << ". Supported algorithms are un (for univariate), sa (for simulation annealing) and hc (for hillclimbing)" << std::endl;
        return 1;
    }
}
