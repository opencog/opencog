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
#include "../combo/eval.h"
#include "../combo/table.h"
#include "../combo/type_tree.h"
#include "../ant_combo_vocabulary/ant_combo_vocabulary.h"

using namespace std;
using namespace opencog;
using namespace ant_combo;
using namespace reduct;

int main()
{
    MT19937RandGen rng(1);

    combo_tree tr;

    while (cin.good()) {
        cin >> tr;
        if (!cin.good())
            break;

        //determine the type of tr
        type_tree tr_type = infer_type_tree(tr);
        cout << "Type : " << tr_type << endl;

        bool ct = is_well_formed(tr_type);

        if (!ct) {
            cout << "Bad type" << endl;
            break;
        }

        int ca = contin_arity(tr_type);
        int s = sample_count(ca);

        //produce random inputs
        contin_input_table cti(s, ca, rng);
        //print cti, for debugging
        cout << "Rnd matrix :" << endl << cti;

        try {
            //evalutate tr over cti and fill mt1
            mixed_table mt1(tr, cti, tr_type, rng);
            //print mt1, for debugging
            cout << "MT1" << endl << mt1 << endl;

            //print the tree before reduction, for debugging
            cout << "Before : " << tr << endl;

            full_reduce(tr, rng);

            //evaluate tr over cti and fill mt2
            mixed_table mt2(tr, cti, tr_type, rng);
            //print mt2, for debugging
            cout << "MT2" << endl << mt2 << endl;

            cout << "After  : " << tr << endl;
            if (mt1 != mt2) {
                cout << mt1 << endl << mt2 << endl;
                cerr << "mixed-tables don't match!" << endl;
                return 1;
            }
        } catch (EvalException& e) {
            cout << e.get_message() << " : " << e.get_vertex() << endl;
        }
    }
    return 0;
}
