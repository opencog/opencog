#include "ComboReduct/combo/vertex.h"
#include "ComboReduct/reduct/reduct.h"
#include "ComboReduct/combo/eval.h"
#include "ComboReduct/combo/type_tree.h"
#include <LADSUtil/mt19937ar.h>
#include "ComboReduct/ant_combo_vocabulary/ant_combo_vocabulary.h"

#include <iostream>

using namespace std;
using namespace ant_combo;
using namespace reduct;

int main() {

  combo_tree tr;  

  LADSUtil::MT19937RandGen rng(0);

  //bool b=is_random(id::random_drinkable); 
  //cout << "is random" << b << endl;

  //b=is_random(id::nearest_movable);
  perception p = instance(id::is_food_ahead);
  builtin_action ba1 = instance(id::move_forward);
  builtin_action ba2 = instance(id::turn_left);
  //cout << "is random" << b << endl;

  cout << "1----------------" << endl;

  cout << "arity " << p->arity() << endl;
  cout << "type tree: " << p->get_type_tree() << endl;
  cout << "output type " << p->get_output_type_tree() << endl;

  cout << "2----------------" << endl;

  cout << "arity " << (int)get_arity(id::sequential_and) << endl;
  cout << "type tree: " << get_type_tree(id::sequential_and) << endl;
  cout << "output type " << get_output_type_tree(id::sequential_and) << endl;
  cout << "arg " << 0 << get_input_type_tree(id::sequential_and,0) << endl;
  cout << "arg " << 1 << get_input_type_tree(id::sequential_and,1) << endl;
  cout << "arg " << 2 << get_input_type_tree(id::sequential_and,2) << endl;

  cout << "3----------------" << endl;

  cout << "arity " << (int)get_arity(id::action_success) << endl;
  cout << "type tree: " << get_type_tree(id::action_success) << endl;
  cout << "output type " << get_output_type_tree(id::action_success) << endl;
  cout << "arg " << 0 << get_input_type_tree(id::action_success,0) << endl;
  cout << "arg " << 1 << get_input_type_tree(id::action_success,1) << endl;
  cout << "arg " << 2 << get_input_type_tree(id::action_success,2) << endl;

  cout << "4----------------" << endl;

  cout << "arity " << ba1->arity() << endl;
  cout << "type tree: " << ba1->get_type_tree() << endl;
  cout << "output type " << ba1->get_output_type_tree() << endl;

  cout << "5----------------" << endl;

  cout << "arity " << ba2->arity() << endl;
  cout << "type tree: " << ba2->get_type_tree() << endl;
  cout << "output type " << ba2->get_output_type_tree() << endl;

  cout << "6----------------" << endl;

  cout << "arity " << (int)get_arity(id::boolean_if) << endl;
  cout << "type tree: " << get_type_tree(id::boolean_if) << endl;
  cout << "output type " << get_output_type_tree(id::boolean_if) << endl;
  cout << "arg " << 0 << get_input_type_tree(id::boolean_if,0) << endl;
  cout << "arg " << 1 << get_input_type_tree(id::boolean_if,1) << endl;
  cout << "arg " << 2 << get_input_type_tree(id::boolean_if,2) << endl;
  
  cout << "----------------" << endl;

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
    int s = sample_count(ca);

    //produce random inputs
    RndNumTable rnt(s, ca, rng);
    //print rnt, for debugging
    cout << "Rnd matrix :" << endl << rnt;

    try {
      //evalutate tr over rnt and fill mat1
      mixed_action_table mat1(tr, rnt, tr_type, rng);
      //print mat1, for debugging
      cout << "MAT1" << endl << mat1 << endl;

      //print the tree before reduction, for debugging
      cout << "Before : " << tr << endl;

      action_reduce(tr);

      //evaluate tr over rnt and fill mat2
      mixed_action_table mat2(tr, rnt, tr_type, rng);
      //print mat2, for debugging
      cout << "MAT2" << endl << mat2 << endl;

      cout << "After  : " << tr << endl;
      if (mat1!=mat2) {
	cout << mat1 << endl << mat2 << endl;
	cerr << "mixed-action-tables don't match!" << endl;
	return 1;
      }
    }
    catch(EvalException& e) {
      cout << e.get_message() << " : " << e.get_vertex() << endl;
    }
  }
  return 0;
}
