/** moses-contin-table.cc --- 
 *
 * Copyright (C) 2010 Alesis Novik
 *
 * Author: Alesis Novik
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

#include <opencog/util/mt19937ar.h>
#include <opencog/util/Logger.h>

#include <opencog/comboreduct/combo/eval.h>

#include <opencog/learning/moses/moses/moses.h>
#include <opencog/learning/moses/moses/optimization.h>
#include <opencog/learning/moses/moses/scoring_functions.h>
#include <opencog/learning/moses/moses/scoring.h>

using namespace moses;
using namespace reduct;
using namespace boost;
using namespace std;

int main(int argc,char** argv) { 

    //set flag to print only cassert and other ERROR level logs on stdout
    opencog::logger().setPrintErrorLevelStdout();

    int arity, max_evals,rand_seed;

    try {
        if (argc!=3)
            throw "foo";
        max_evals=lexical_cast<int>(argv[1]);
    } catch (...) {
        cerr << "usage: " << argv[0] << " maxevals file_with_table" << endl;
        exit(1);
    }


    //Seed and able reading from file
    ifstream in(argv[2]);

    in >> rand_seed;
    opencog::MT19937RandGen rng(rand_seed);

    contin_table contintable;
    contin_table_inputs inputtable;
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
    arity = inputtable[0].size();

    type_tree tt(id::lambda_type);
    tt.append_children(tt.begin(),id::contin_type,arity + 1);

    contin_score_sqr score(contintable, inputtable, rng);
    contin_bscore bscore(contintable, inputtable, rng);

    metapopulation<contin_score_sqr,contin_bscore,univariate_optimization> 
        metapop(rng, combo_tree(id::plus), tt,
                contin_reduction(vertex_set(), rng), true,
                score, bscore, true, univariate_optimization(rng));
    
    moses::moses(metapop,max_evals,0);
}
