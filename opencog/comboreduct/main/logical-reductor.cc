/*
 * opencog/comboreduct/main/logical-reductor.cc
 *
 * Copyright (C) 2002-2008 Novamente LLC
 * All Rights Reserved
 *
 * Written by Moshe Looks
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
#include <opencog/comboreduct/reduct/reduct.h>
#include <opencog/comboreduct/combo/eval.h>
#include <iostream>
#include <opencog/util/mt19937ar.h>
#include <opencog/comboreduct/ant_combo_vocabulary/ant_combo_vocabulary.h>

using namespace ant_combo;
using namespace std;
using namespace reduct;
using namespace opencog;

int main()
{

    MT19937RandGen rng(0);

    combo_tree tr;
    while (cin.good()) {
        cin >> tr;
        if (!cin.good())
            break;
        truth_table tt1(tr, rng);
        //cout << "AR" << endl;
        logical_reduce(tr);
        //cout << "RA" << endl;
        truth_table tt2(tr, integer_log2(tt1.size()), rng);
        cout << tr << endl;
        //cout << "checking tt" << endl;
        if (tt1 != tt2) {
            cout << tt1 << endl << tt2 << endl;
            cerr << "truth-tables don't match!" << endl;
            return 1;
        }
        //cout << "OK" << endl;
    }
    return 0;
}
