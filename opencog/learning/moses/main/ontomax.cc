/*
 * opencog/learning/moses/main/ontomax.cc
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
#include "edaopt.h"

#include <opencog/learning/moses/eda/initialization.h>
#include <opencog/util/mt19937ar.h>
#include <opencog/util/Logger.h>

using std::string;
using std::vector;
using boost::lexical_cast;
using namespace opencog;
using namespace moses;

void recbuild(onto_tree& tr,onto_tree::iterator it,
	      int b,int maxd,int d,int s) {
    *it=lexical_cast<string>(d)+lexical_cast<string>(s);
    if (d<maxd) {
        tr.append_children(it,b);
        int child_s=0;
        for (onto_tree::sibling_iterator sib=it.begin();sib!=it.end();++sib)
            recbuild(tr,sib,b,maxd,d+1,s*b+child_s++);
    }
}

int main(int argc,char** argv) { 

    //set flag to print only cassert and other ERROR level logs on stdout
    logger().setPrintErrorLevelStdout();

    vector<string> addition_args{"depth", "branching"};
    optargs args(argc, argv, addition_args);
    int depth=lexical_cast<int>(argv[5]);
    int branching=lexical_cast<int>(argv[6]);
    cout_log_best_and_gen logger;
    
    MT19937RandGen rng(args.rand_seed);
    
    onto_tree tr("");
    recbuild(tr,tr.begin(),branching,depth,0,0);
    field_set fs(field_set::onto_spec(tr),args.length);
    instance_set<contin_t> population(args.popsize,fs);
    foreach(instance& inst,population) {
        occam_randomize_onto(fs,inst,rng);
    }

    optimize(population,args.n_select,args.n_generate,args.max_gens,
             ontomax(fs),
             terminate_if_gte<contin_t>((depth+pow(float(branching),
                                                   depth)-1)*args.length),
             tournament_selection(2,rng),
             univariate(),local_structure_probs_learning(),
             replace_the_worst(),logger,rng);
}
