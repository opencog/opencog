/*
 * opencog/learning/moses/main/moses-sregression.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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

#include <boost/lexical_cast.hpp>

#include <opencog/comboreduct/reduct/reduct.h>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/Logger.h>

#include <opencog/learning/moses/moses/moses.h>
#include <opencog/learning/moses/moses/optimization.h>
#include <opencog/learning/moses/moses/scoring_functions.h>


using namespace moses;
using namespace reduct;
using namespace boost;
using namespace std;

int main(int argc,char** argv) { 
    //set flag to print only cassert and other ERROR level logs on stdout
    opencog::logger().setPrintErrorLevelStdout();
    
    int order,max_evals,rand_seed;
    int arity=1,nsamples=20;
    try {
        if (argc!=4)
            throw "foo";
        rand_seed=lexical_cast<int>(argv[1]);
        order=lexical_cast<int>(argv[2]);
        max_evals=lexical_cast<int>(argv[3]);
    } catch (...) {
        cerr << "usage: " << argv[0] << " seed order maxevals" << endl;
        exit(1);
    }
    
    opencog::MT19937RandGen rng(rand_seed);
    
    type_tree tt(id::lambda_type);
    tt.append_children(tt.begin(),id::contin_type,2);
    
    contin_table_inputs rands(nsamples,arity,rng);
    
    cout << rands << endl;
    
    metapopulation<contin_score,contin_bscore,univariate_optimization> 
        metapop(rng, combo_tree(id::plus),
                tt,contin_reduction(rng), true,
                contin_score(simple_symbolic_regression(order),rands, rng),
                contin_bscore(simple_symbolic_regression(order),rands, rng),
                true,
                univariate_optimization(rng));
    
    //had to put namespace moses otherwise gcc-4.1 complains that it is ambiguous
    moses::moses(metapop,max_evals,0);
}
