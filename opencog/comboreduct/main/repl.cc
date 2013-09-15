#include <iostream>
#include <fstream>

#include "../interpreter/eval.h"
#include "../combo/procedure_repository.h"
#include "../ant_combo_vocabulary/ant_combo_vocabulary.h"

using namespace ant_combo;
using namespace std;
using namespace opencog;
using namespace opencog::combo;



int main()
{
    procedure_repository repo;
    ifstream file("procedures.combo");
    int n = load_procedure_repository<ant_builtin_action, ant_perception, ant_action_symbol, ant_indefinite_object>
        (file, repo, false);
    cout << n << " procedures loaded" << endl;

    combo_tree tr;
    vertex_seq empty;

    while (cin.good()) {
        cout << "combo_repl> ";
        cin >> tr;
        if (!cin.good())
            break;

        //ProcedureEvaluator* pe = new ProcedureEvaluator(tr);

        combo_tree trv;
        try {
            combo_tree::pre_order_iterator it = tr.begin();
            repo.instantiate_procedure_calls(tr, true);
            //pe->
            trv = eval_throws_tree(empty, it);
        }
        catch (StandardException e) {
            cout << "Exception: " << e.what() << endl;
        }
        catch (OverflowException e) {
            cout << "Exception: " << e.get_message() << endl;
        }
        catch (EvalException e) {
            cout << "Exception: " << e.get_message() << endl;
        }
        cout << trv << endl;

        //delete pe;
  }

  return 0;
}
