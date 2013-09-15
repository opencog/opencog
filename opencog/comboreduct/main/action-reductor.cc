/*
 * opencog/comboreduct/main/action-reductor.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Nil Geisweiller
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <iostream>

#include <opencog/util/mt19937ar.h>

#include "../combo/vertex.h"
#include "../reduct/reduct.h"
#include "../interpreter/eval.h"
#include "../table/table.h"
#include "../table/table_io.h"
#include "../type_checker/type_tree.h"
#include "../ant_combo_vocabulary/ant_combo_vocabulary.h"

using namespace std;
using namespace opencog;
using namespace ant_combo;
using namespace reduct;

int main()
{

    combo_tree tr;

    randGen().seed(0);

    //bool b=is_random(id::random_drinkable);
    //cout << "is random" << b << endl;

    //b=is_random(id::nearest_movable);
    perception p = get_instance(id::is_food_ahead);
    builtin_action ba1 = get_instance(id::move_forward);
    builtin_action ba2 = get_instance(id::turn_left);
    //cout << "is random" << b << endl;

    cout << "1----------------" << endl;

    cout << "arity " << p->arity() << endl;
    cout << "type tree: " << p->get_type_tree() << endl;
    cout << "output type " << p->get_output_type_tree() << endl;

    cout << "2----------------" << endl;

    cout << "arity " << (int)get_arity(id::sequential_and) << endl;
    cout << "type tree: " << get_type_tree(id::sequential_and) << endl;
    cout << "output type " << get_output_type_tree(id::sequential_and) << endl;
    cout << "arg " << 0 << get_input_type_tree(id::sequential_and, 0) << endl;
    cout << "arg " << 1 << get_input_type_tree(id::sequential_and, 1) << endl;
    cout << "arg " << 2 << get_input_type_tree(id::sequential_and, 2) << endl;

    cout << "3----------------" << endl;

    cout << "arity " << (int)get_arity(id::action_success) << endl;
    cout << "type tree: " << get_type_tree(id::action_success) << endl;
    cout << "output type " << get_output_type_tree(id::action_success) << endl;
    cout << "arg " << 0 << get_input_type_tree(id::action_success, 0) << endl;
    cout << "arg " << 1 << get_input_type_tree(id::action_success, 1) << endl;
    cout << "arg " << 2 << get_input_type_tree(id::action_success, 2) << endl;

    cout << "4----------------" << endl;

    cout << "arity " << ba1->arity() << endl;
    cout << "type tree: " << ba1->get_type_tree() << endl;
    cout << "output type " << ba1->get_output_type_tree() << endl;

    cout << "5----------------" << endl;

    cout << "arity " << ba2->arity() << endl;
    cout << "type tree: " << ba2->get_type_tree() << endl;
    cout << "output type " << ba2->get_output_type_tree() << endl;

#if 0
    // TODO -- replace this by cond
    cout << "6----------------" << endl;

    cout << "arity " << (int)get_arity(id::boolean_if) << endl;
    cout << "type tree: " << get_type_tree(id::boolean_if) << endl;
    cout << "output type " << get_output_type_tree(id::boolean_if) << endl;
    cout << "arg " << 0 << get_input_type_tree(id::boolean_if, 0) << endl;
    cout << "arg " << 1 << get_input_type_tree(id::boolean_if, 1) << endl;
    cout << "arg " << 2 << get_input_type_tree(id::boolean_if, 2) << endl;
#endif

    cout << "----------------" << endl;

    while (cin.good()) {
        cin >> tr;
        if (!cin.good())
            break;

        //determine the type of tr
        type_tree tt = infer_type_tree(tr);
        cout << "Type : " << tt << endl;

        bool ct = is_well_formed(tt);

        if (!ct) {
            cout << "Bad type" << endl;
            break;
        }

        //produce random inputs
        ITable cti(tt);
        //print cti, for debugging
        cout << "Rnd matrix :" << endl << cti;

        try {
            //evalutate tr over cti and fill mat1
            OTable mat1(tr, cti);
            //print mat1, for debugging
            cout << "MAT1" << endl << mat1 << endl;

            //print the tree before reduction, for debugging
            cout << "Before : " << tr << endl;

            action_reduce(tr);

            //evaluate tr over cti and fill mat2
            OTable mat2(tr, cti);
            //print mat2, for debugging
            cout << "MAT2" << endl << mat2 << endl;

            cout << "After  : " << tr << endl;
            if (mat1 != mat2) {
                cout << mat1 << endl << mat2 << endl;
                cerr << "mixed action tables don't match!" << endl;
                return 1;
            }
        } catch (OverflowException& e) {
            cout << e.get_message() << " : " << e.get_vertex() << endl;
        } catch (EvalException& e) {
            cout << e.get_message() << " : " << e.get_vertex() << endl;
        }
    }
    return 0;
}
