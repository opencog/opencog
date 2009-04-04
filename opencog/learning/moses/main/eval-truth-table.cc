#include "moses/moses.h"
#include "moses/scoring_functions.h"

//ant_combo_vocabulary is used only for the boolean core vocabulary
#include "comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h"

using namespace moses;
using namespace ant_combo;

int main(int argc,char** argv) { 
  int arity;
  try {
    if (argc!=2)
      throw "foo";
  } catch (...) {
    cerr << "usage: " << argv[0] << " TestFileName" << endl;
    exit(1);
  }

  ifstream in(argv[1]);

  CaseBasedBoolean bc(in);
  arity=bc.arity();

  combo_tree tr;
  cout << "Enter formula:" << endl;
  cin >> tr;

  cout << "Evaluating formula:" << tr << endl;

  ConfusionMatrix cm=bc.ComputeConfusionMatrix(tr);

  cout << "TP: " << cm.TP << "  FN: " << cm.FN << endl;  
  cout << "FP: " << cm.FP << "  TN: " << cm.TN << endl;  

  cout << "Accuracy:  " << (float)(cm.TP+cm.TN)/(cm.FP+cm.TP+cm.TN+cm.FN) << endl;

}




