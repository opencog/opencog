/*
 * opencog/learning/moses/main/moses-ant-hillclimbing.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Predrag Janicic
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
#include <algorithm>
#include <iostream>
#include <set>
#include <string>
#include <boost/lexical_cast.hpp>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/numeric.h>

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>
#include <opencog/comboreduct/reduct/reduct.h>

#include "../moses/moses.h"
#include "../optimization/optimization.h"
#include "../moses/scoring_functions.h"
#include "ant_scoring.h"


using namespace opencog;
using namespace moses;
using namespace reduct;
using namespace boost;
using namespace ant_combo;
using namespace std;


//typedef std::set<combo::vertex> operator_set;
//typedef operator_set::iterator operator_set_it;  

//typedef std::set<combo::combo_tree, size_tree_order<combo::vertex> > 
//    combo_tree_ns_set;

int main(int argc,char** argv) { 
  int max_evals,rand_seed;
  try {
    if (argc!=3)
      throw "foo";
    rand_seed=lexical_cast<int>(argv[1]);
    max_evals=atoi(argv[2]);
  } catch (...) {
    cerr << "usage: " << argv[0] << " seed maxevals" << endl;
    exit(1);
  }

  type_tree tt(id::lambda_type);
  tt.append_children(tt.begin(),id::action_result_type,1);

  ant_score scorer;
  ant_bscore bscorer;

  MT19937RandGen rng(rand_seed);

  operator_set ignore_ops;
  combo_tree_ns_set perceptions;
  combo_tree_ns_set actions;

  actions.insert(combo_tree(get_instance(id::turn_left)));
  actions.insert(combo_tree(get_instance(id::turn_right)));
  actions.insert(combo_tree(get_instance(id::move_forward)));

  perceptions.insert(combo_tree(get_instance(id::is_food_ahead)));

/*
  metapopulation<ant_score,ant_bscore,iterative_hillclimbing> 
    metapop(rng,combo_tree(id::sequential_and),tt,action_reduction(),
	    scorer,
	    bscorer,
            iterative_hillclimbing(rng));
  //had to put namespace moses otherwise gcc-4.1 complains that it is ambiguous
  moses::moses(metapop,max_evals,0,&os,&perceptions,&actions,op);
*/

/*
  metapopulation<ant_score,ant_bscore,univariate_optimization> 
    metapop(rng,combo_tree(id::sequential_and),tt,action_reduction(),
	    scorer,
	    bscorer,
            univariate_optimization(rng));
  moses::moses(metapop,max_evals,0,&os,&perceptions,&actions,op);
*/


  metapopulation<ant_score, ant_bscore, iterative_hillclimbing> 
      metapop(rng, combo_tree(id::sequential_and), tt, action_reduction(),
              scorer, bscorer, iterative_hillclimbing(rng));
  
  moses_parameters moses_param(max_evals, -1, 0,
                               ignore_ops, &perceptions, &actions);
  moses::moses(metapop, moses_param);
}

