#include <iostream>

#include "../interpreter/eval.h"
#include "../ant_combo_vocabulary/ant_combo_vocabulary.h"

using namespace ant_combo;
using namespace std;
using namespace opencog;

int main()
{
    combo_tree tr;
    vertex_seq empty;

    while (cin.good()) {
        cout << "combo_repl> ";
        cin >> tr;
        if (!cin.good())
            break;

        combo_tree trv;
        try {
            trv = eval_throws_tree(empty, tr);
        }
        catch (StandardException e) {
            cout << "Exception: " << e.what() << endl;
        }
        cout << trv << endl;
  }

  return 0;
}
