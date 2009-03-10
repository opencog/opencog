#include "main/edaopt.h"
#include "eda/initialization.h"
#include "util/mt19937ar.h"

int main(int argc,char** argv) { 
  cassert(TRACE_INFO, argc==6);
  optargs args(argc,argv);
  int depth=lexical_cast<int>(argv[5]);
  cout_log_best_and_gen logger;

  opencog::MT19937RandGen rng(args.rand_seed);

  /*field_set fs(field_set::spec(field_set::contin_spec(2.0,2.5,0.5,depth),
    args.length));*/
  field_set fs(field_set::contin_spec(0.0,0.5,0.5,depth),args.length);
  instance_set<contin_t> population(args.popsize,fs);
  foreach(instance& inst,population) {
    occam_randomize_contin(fs,inst,rng);
    cout << fs.stream(inst) << endl;
    cout << fs.stream_raw(inst) << endl;
  }

  contin_t epsilon=fs.contin().front().epsilon();
  optimize(population,args.n_select,args.n_generate,args.max_gens,
	   sphere(fs),
	   terminate_if_gte<contin_t>(-args.length*epsilon),
	   //terminate_if_gte(args.length*(7-2*epsilon)*(7-2*epsilon)),
	   tournament_selection(2, rng),
	   univariate(),local_structure_probs_learning(),
	   replace_the_worst(),logger, rng);
}
