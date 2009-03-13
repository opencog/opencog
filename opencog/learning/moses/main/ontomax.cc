#include "main/edaopt.h"
#include "eda/initialization.h"
#include "util/mt19937ar.h"

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
  opencog::cassert(TRACE_INFO, argc==7);
  optargs args(argc,argv," depth branching");
  int depth=lexical_cast<int>(argv[5]);
  int branching=lexical_cast<int>(argv[6]);
  cout_log_best_and_gen logger;

  opencog::MT19937RandGen rng(args.rand_seed);
  
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
