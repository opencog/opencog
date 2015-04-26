/*
 * opencog/comboreduct/main/perception-reduct.cc
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

using namespace std;
using namespace opencog;
using namespace reduct;

int main()
{

    combo_tree tr;

    randGen().seed(0);

    while (cin.good()) {
        cin >> tr;
        if (!cin.good())
            break;

        //get type_node tree of tr.
        type_tree tt = infer_type_tree(tr);
        cout << "Type : " << tt << endl;

        //check whether the tree is well-formed.
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
            //mixed_action_table mat1(tr, cti, tr_type);
            //print mat1, for debugging
            //cout << "MAT1" << endl << mat1 << endl;

            //print the tree before reduction, for debugging
            cout << "Before : " << tr << endl;

            perception_reduce(tr);

            //evaluate tr over cti and fill mat2
            //mixed_action_table mat2(tr, cti, tr_type);
            //print mat2, for debugging
            //cout << "MAT2" << endl << mat2 << endl;

            cout << "After  : " << tr << endl;
            //if (mat1!=mat2) {
            //cout << mat1 << endl << mat2 << endl;
            //cerr << "mixed-perception-tables don't match!" << endl;
            //return 1;
            //}
        } catch (OverflowException& e) {
            cout << e.get_message() << " : " << e.get_vertex() << endl;
        } catch (EvalException& e) {
            cout << e.get_message() << " : " << e.get_vertex() << endl;
        }
    }
    return 0;
}
