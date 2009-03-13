#include "main/edaopt.h"
#include "util/mt19937ar.h"

int main(int argc,char** argv) { 
  opencog::cassert(TRACE_INFO, argc==6);
  optargs args(argc,argv);
  int n=lexical_cast<int>(argv[5]);
  cout_log_best_and_gen logger;

  opencog::MT19937RandGen rng(args.rand_seed);

  field_set fs(field_set::disc_spec(n),args.length); //all n-arry
  instance_set<int> population(args.popsize,fs);
  foreach(instance& inst,population)
    generate(fs.begin_disc(inst),fs.end_disc(inst),bind(&opencog::RandGen::randint, ref(rng), n));

  optimize(population,args.n_select,args.n_generate,args.max_gens,n_max(fs),
	   terminate_if_gte<int>((n-1)*args.length),tournament_selection(2,rng),
	   univariate(),local_structure_probs_learning(),
	   replace_the_worst(),logger,rng);
}
