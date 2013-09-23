/*
 * opencog/comboreduct/main/contin-reductor.cc
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

#include "../reduct/reduct.h"
#include "../interpreter/eval.h"
#include "../table/table.h"
#include "../table/table_io.h"

using namespace std;
using namespace opencog;
using namespace reduct;

int main()
{
    randGen().seed(1);
    int reduct_effort = 2;
    combo_tree tr;

    while (cin.good()) {
        cin >> tr;
        if (!cin.good())
            break;

        type_tree tt = infer_type_tree(tr);

        //produce random inputs
        ITable cti(tt);

        try {

            //print cti, for debugging
            cout << "Rnd matrix :" << endl << cti;


            //evalutate tr over cti and fill ct1
            OTable ct1(tr, cti);

            //print the tree before reduction, for debugging
            cout << "Before : " << tr << endl;

            contin_reduce(tr, reduct_effort, vertex_set());

            //evaluate tr over cti and fill ct2
            OTable ct2(tr, cti);

            cout << "After  : " << tr << endl;
            if (ct1 != ct2) {
                cout << ct1 << endl << ct2 << endl;
                cerr << "contin tables don't match!" << endl;
                //return 1;
            }
        } catch (OverflowException& e) {
            cout << e.get_message() << " : " << e.get_vertex() << endl;
        } catch (EvalException& e) {
            cout << e.get_message() << " : " << e.get_vertex() << endl;
        }
    }
    return 0;
}
