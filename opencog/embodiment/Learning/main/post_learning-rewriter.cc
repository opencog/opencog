#include "comboreduct/combo/vertex.h"
#include "comboreduct/combo/type_tree.h"

#include "RewritingRules/RewritingRules.h"
#include "PetComboVocabulary.h"

#include <iostream>

using namespace std;
using namespace PetCombo;
using namespace reduct;

int main() {

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

    //print the tree before reduction, for debugging
    cout << "Before : " << tr << endl;

    post_learning_rewrite(tr);

    cout << "After  : " << tr << endl;

  }
}
