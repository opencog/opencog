#include "ComboReduct/reduct/reduct.h"
#include "ComboReduct/combo/eval.h"
#include "ComboReduct/combo/type_tree.h"
#include <iostream>
#include <LADSUtil/mt19937ar.h>
#include "ComboReduct/ant_combo_vocabulary/ant_combo_vocabulary.h"

using namespace std;
using namespace ant_combo;
using namespace reduct;
using namespace LADSUtil;

int main() {
  MT19937RandGen rng(1);

  combo_tree tr;

  while (cin.good()) {
    cin >> tr;
    if (!cin.good())
      break;

    //determine the type of tr
    type_tree tr_type = infer_type_tree(tr);
    cout << "Type : " << tr_type << endl;

    bool ct = is_well_formed(tr_type);

    if(!ct) {
      cout << "Bad type" << endl;
      break;
    }

    int ca = contin_arity(tr_type);
    int s = sample_count(ca);//return always 5, for the moment
                             //TODO, maybe not contin_arity but arity
    //produce random inputs
    RndNumTable rnt(s, ca, rng);
    //print rnt, for debugging
    cout << "Rnd matrix :" << endl << rnt;

    try {
      //evalutate tr over rnt and fill mt1
      mixed_table mt1(tr, rnt, tr_type, rng);
      //print mt1, for debugging
      cout << "MT1" << endl << mt1 << endl;

      //print the tree before reduction, for debugging
      cout << "Before : " << tr << endl;

      mixed_reduce(tr,rng);

      //evaluate tr over rnt and fill mt2
      mixed_table mt2(tr, rnt, tr_type, rng);
      //print mt2, for debugging
      cout << "MT2" << endl << mt2 << endl;

      cout << "After  : " << tr << endl;
      if (mt1!=mt2) {
	cout << mt1 << endl << mt2 << endl;
	cerr << "mixed-tables don't match!" << endl;
	return 1;
      }
    }
    catch(EvalException& e) {
      cout << e.get_message() << " : " << e.get_vertex() << endl;
    }
  }
  return 0;
}
