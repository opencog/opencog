#include <iostream>

#include "../combo/eval.h"
#include "../ant_combo_vocabulary/ant_combo_vocabulary.h"

using namespace ant_combo;
using namespace std;
using namespace opencog;

int main()
{
    combo_tree tr;
    vertex_seq empty;

    while (cin.good()) {
        cout<<"combo_repl> ";
        cin >> tr;
        if (!cin.good())
            break;
        vertex v = eval_throws_binding(empty, tr);
        cout << v << endl;
  }

  return 0;
}
