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
#include "../combo/eval.h"
#include "../combo/table.h"

using namespace std;
using namespace opencog;
using namespace reduct;

int main()
{
    MT19937RandGen rng(1);

    combo_tree tr;

    while (cin.good()) {
        cin >> tr;
        if (!cin.good())
            break;

        type_tree tt = infer_type_tree(tr);

        //produce random inputs
        ITable cti(tt, rng);

        try {

            //print cti, for debugging
            cout << "Rnd matrix :" << endl << cti;


            //evalutate tr over cti and fill ct1
            OTable ct1(tr, cti, rng);

            //print the tree before reduction, for debugging
            cout << "Before : " << tr << endl;

            contin_reduce(tr, vertex_set(), rng);

            //evaluate tr over cti and fill ct2
            OTable ct2(tr, cti, rng);

            cout << "After  : " << tr << endl;
            if (ct1 != ct2) {
                cout << ct1 << endl << ct2 << endl;
                cerr << "contin tables don't match!" << endl;
                //return 1;
            }
        } catch (EvalException& e) {
            cout << e.get_message() << " : " << e.get_vertex() << endl;
        }
    }
    return 0;
}
