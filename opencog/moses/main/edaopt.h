#include "MosesEda/eda/scoring_functions.h"
#include "MosesEda/eda/termination.h"
#include "MosesEda/eda/local_structure.h"
#include "MosesEda/eda/replacement.h"
#include "MosesEda/eda/logging.h"
#include "MosesEda/eda/optimize.h"

#include <LADSUtil/selection.h>

#include <boost/lexical_cast.hpp>

using namespace eda;
using namespace std;
using namespace LADSUtil;
using namespace boost;

struct optargs {
  optargs(int argc,char** argv,string usage="") { 
    if (argc<5) {
      cerr << "not enough args, usage: " << argv[0] 
	   << " seed length popsize ngens " << usage << endl;
      exit(1);
    }
    try {
      assert(argc>=5);
      rand_seed=lexical_cast<int>(argv[1]);
      length=lexical_cast<int>(argv[2]);
      popsize=lexical_cast<int>(argv[3]);
      n_select=popsize;
      n_generate=popsize/2;
      max_gens=lexical_cast<int>(argv[4]);
    } catch (...) {
      cerr << "invalid args, usage: " << argv[0]
	   << " seed length popsize ngens " << usage << endl;
      exit(1);
    }
  }
  int rand_seed;
  int length;
  int popsize;
  int n_select;
  int n_generate;
  int max_gens;
};
