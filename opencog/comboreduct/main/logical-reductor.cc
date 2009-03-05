#include "ComboReduct/reduct/reduct.h"
#include "ComboReduct/combo/eval.h"
#include <iostream>
#include <LADSUtil/mt19937ar.h>
#include "ComboReduct/ant_combo_vocabulary/ant_combo_vocabulary.h"

using namespace ant_combo;
using namespace std;
using namespace reduct;
using namespace LADSUtil;

int main() {

  MT19937RandGen rng(0);

  combo_tree tr;
  while (cin.good()) {
    cin >> tr;
    if (!cin.good())
      break;
    truth_table tt1(tr, rng);
    //cout << "AR" << endl;
    logical_reduce(tr);        
    //cout << "RA" << endl;
    truth_table tt2(tr,integer_log2(tt1.size()), rng);
    cout << tr << endl;
    //cout << "checking tt" << endl;
    if (tt1!=tt2) {
      cout << tt1 << endl << tt2 << endl;
      cerr << "truth-tables don't match!" << endl;
      return 1;
    }
    //cout << "OK" << endl;
  }
  return 0;
}
