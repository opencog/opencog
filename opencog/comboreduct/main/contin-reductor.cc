#include "ComboReduct/reduct/reduct.h"
#include "ComboReduct/combo/eval.h"
#include <iostream>
#include <LADSUtil/mt19937ar.h>
#include "ComboReduct/ant_combo_vocabulary/ant_combo_vocabulary.h"

using namespace std;
using namespace reduct;
using namespace LADSUtil;
using namespace ant_combo;

int main() {
  MT19937RandGen rng(1);

  combo_tree tr;

  while (cin.good()) {
    cin >> tr;
    if (!cin.good())
      break;

    int a = arity(tr);
    int s = sample_count(a);//return always 5, for the moment

    //produce random inputs
    RndNumTable rnt(s, a, rng);

    try {

      //print rnt, for debugging
      cout << "Rnd matrix :" << endl << rnt;


      //evalutate tr over rnt and fill ct1
      contin_table ct1(tr, rnt, rng);

      //print the tree before reduction, for debugging
      cout << "Before : " << tr << endl;

      contin_reduce(tr,rng);

      //evaluate tr over rnt and fill ct2
      contin_table ct2(tr, rnt, rng);

      cout << "After  : " << tr << endl;
      if (ct1!=ct2) {
	cout << ct1 << endl << ct2 << endl;
	cerr << "contin-tables don't match!" << endl;
	//return 1;
      }
    }
    catch(EvalException& e) {
      cout << e.get_message() << " : " << e.get_vertex() << endl;
    }
  }
  return 0;
}
