/** moses-ann-pole1.cc --- 
 *
 * Copyright (C) 2010-2011 OpenCog Foundation
 *
 * Author: Joel Lehman
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
#include <opencog/comboreduct/interpreter/eval.h>

#include "../deme/deme_expander.h"
#include "../metapopulation/metapopulation.h"
#include "../moses/moses_main.h"
#include "../representation/representation.h"
#include "../optimization/optimization.h"
#include "../scoring/scoring_base.h"

#include "pole_scoring.h"

using namespace std;
using namespace boost;
using namespace opencog;
using namespace moses;
using namespace reduct;

int main(int argc, char** argv)
{

    // Set flag to print only cassert and other ERROR level logs on stdout.
    logger().setPrintErrorLevelStdout();

    // Read in maximum evaluations and RNG seed from command line.
    int seed;
    bool reduce=true;
    try
    {
        //if (argc!=3)
        //    throw "foo";
        // int max_evals=lexical_cast<int>(argv[1]);
        seed=lexical_cast<int>(argv[2]);
        
        set_stepsize(lexical_cast<double>(argv[3]));
        set_expansion(lexical_cast<double>(argv[4]));
        set_depth(lexical_cast<int>(argv[5]));

        reduce = lexical_cast<int>(argv[6]);
    }
    catch (...)
    {
        cerr << "usage: " << argv[0] << " maxevals seed" << endl;
        exit(1);
    }
    
    // Read seed tree in from stdin.
    combo_tree tr;
    cin >> tr; 

    randGen().seed(seed);

    type_tree tt(id::lambda_type);
    tt.append_children(tt.begin(), id::ann_type, 1);
    
    const reduct::rule* si = &(ann_reduction());
    if(!reduce)
        si = &(clean_reduction());

    //SINGLE MARKOVIAN POLE TASK
    ann_pole_bscore p_bscore; 
    behave_cscore cscorer(p_bscore);

    univariate_optimization optim_algo;
    deme_expander dex(tt, *si,  *si, cscorer, optim_algo);
    metapopulation metapop_pole(tr, cscorer);

    moses_parameters pa;
    moses_statistics st;
    run_moses(metapop_pole, dex, pa, st);

    //change best tree into ANN
    tree_transform trans; 
    combo_tree best = metapop_pole.best_tree();
    ann bestnet = trans.decodify_tree(best);
   
    //write out the best network
    cout << "Best network: " << endl; 
    cout << &bestnet << endl;

    //write out in dot format
    bestnet.write_dot("best_nn.dot");
    
    //for parameter sweep
    cout << metapop_pole.best_score() << endl;
}
