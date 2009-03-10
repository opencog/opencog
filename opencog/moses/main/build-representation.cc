#include "moses/moses.h"
#include "moses/optimization.h"
#include "moses/scoring_functions.h"
#include <boost/lexical_cast.hpp>
#include "comboreduct/reduct/reduct.h"
#include <iostream>
#include "util/mt19937ar.h"

using namespace moses;
using namespace reduct;
using namespace boost;
using namespace std;

int main(int argc,char** argv) { 
    combo_tree tr;
  opencog::MT19937RandGen rng(0);
  while (cin.good()) {
    cin >> tr;
    if (!cin.good())
      break;
    
    logical_reduce(tr);

    representation rep(logical_reduction(),tr,infer_type_tree(tr),rng);
    
    /*combo_tree tmp(rep.exemplar());

    for (int i=0;i<10;++i) { 
      cout << rep.exemplar() << endl;

      eda::instance inst(rep.fields().packed_width());
      for (eda::field_set::disc_iterator it=rep.fields().begin_raw(inst);
	   it!=rep.fields().end_raw(inst);++it)
	it.randomize();	
      
      cout << rep.fields().stream(inst) << endl;
      rep.transform(inst);
      cout << rep.exemplar() << endl;

      rep.clear_exemplar();
      cassert(TRACE_INFO, tmp==rep.exemplar());
      }*/
  }
}
