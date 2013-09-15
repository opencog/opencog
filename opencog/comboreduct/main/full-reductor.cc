/*
 * opencog/comboreduct/main/full-reductor.cc
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
#include "../type_checker/type_tree.h"

using namespace std;
using namespace opencog;
using namespace reduct;

int main()
{
    randGen().seed(1);

    combo_tree tr;

    while (cin.good()) {
        cin >> tr;
        if (!cin.good())
            break;

        // determine the type of tr
        type_tree tt = infer_type_tree(tr);
        cout << "Type : " << tt << endl;

        bool ct = is_well_formed(tt);

        if (!ct) {
            cout << "Bad type" << endl;
            break;
        }

        // produce random inputs
        ITable cti(tt);
        // print cti, for debugging
        cout << "Rnd matrix :" << endl << cti;

        try {
            // evalutate tr over cti and fill mt1
            OTable mt1(tr, cti);
            //print mt1, for debugging
            cout << "MT1" << endl << mt1 << endl;

            //print the tree before reduction, for debugging
            cout << "Before : " << tr << endl;

            full_reduce(tr);

            // evaluate tr over cti and fill mt2
            OTable mt2(tr, cti);
            //print mt2, for debugging
            cout << "MT2" << endl << mt2 << endl;

            cout << "After  : " << tr << endl;
            if (mt1 != mt2) {
                cout << mt1 << endl << mt2 << endl;
                cerr << "mixed tables don't match!" << endl;
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
