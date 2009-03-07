#include "comboreduct/combo/vertex.h"
#include "comboreduct/reduct/reduct.h"
#include "comboreduct/combo/eval.h"
#include "comboreduct/combo/type_tree.h"
#include "util/mt19937ar.h"
#include "comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h"

#include <iostream>

using namespace std;
using namespace ant_combo;
using namespace reduct;

int main() {

  combo_tree tr;  

  opencog::MT19937RandGen rng(0);

  while (cin.good()) {
    cin >> tr;
    if (!cin.good())
      break;

    //get type_node tree of tr.
    type_tree tr_type = infer_type_tree(tr);
    cout << "Type : " << tr_type << endl;

    //check whether the tree is well-formed.
    bool ct = is_well_formed(tr_type);
    
    if(!ct) {
      cout << "Bad type" << endl;
      break;
    }
    
    int ca = contin_arity(tr_type);
    int s = sample_count(ca);

    //produce random inputs
    RndNumTable rnt(s, ca, rng);
    //print rnt, for debugging
    cout << "Rnd matrix :" << endl << rnt;

    try {
      //evalutate tr over rnt and fill mat1
      //mixed_action_table mat1(tr, rnt, tr_type);
      //print mat1, for debugging
      //cout << "MAT1" << endl << mat1 << endl;

      //print the tree before reduction, for debugging
      cout << "Before : " << tr << endl;

      perception_reduce(tr);

      //evaluate tr over rnt and fill mat2
      //mixed_action_table mat2(tr, rnt, tr_type);
      //print mat2, for debugging
      //cout << "MAT2" << endl << mat2 << endl;

      cout << "After  : " << tr << endl;
      //if (mat1!=mat2) {
      //cout << mat1 << endl << mat2 << endl;
      //cerr << "mixed-perception-tables don't match!" << endl;
      //return 1;
      //}
    }
    catch(EvalException& e) {
      cout << e.get_message() << " : " << e.get_vertex() << endl;
    }
  }
  return 0;
}
