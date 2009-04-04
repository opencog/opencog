#include "main/edaopt.h"
#include "util/mt19937ar.h"

int main(int argc,char** argv) { 
  optargs args(argc,argv);
  cout_log_best_and_gen logger;
  field_set fs(field_set::disc_spec(2),args.length); //all boolean

  opencog::MT19937RandGen rng(args.rand_seed);

  instance_set<int> population(args.popsize,fs);
  foreach(instance& inst,population)
    generate(fs.begin_bits(inst),fs.end_bits(inst),bind(&opencog::RandGen::randbool,ref(rng)));

  optimize(population,args.n_select,args.n_generate,args.max_gens,
	   one_max(),terminate_if_gte<int>(args.length),tournament_selection(2, rng),
	   univariate(),local_structure_probs_learning(),
	   replace_the_worst(),logger, rng);
}
