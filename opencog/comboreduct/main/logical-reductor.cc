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
#include <iostream>

#include <opencog/util/mt19937ar.h>
#include <opencog/util/Logger.h>

#include "../reduct/reduct.h"
#include "../interpreter/eval.h"
#include "../table/table.h"
#include "../table/table_io.h"

using namespace std;
using namespace opencog;
using namespace reduct;

int main()
{
    logger().setPrintErrorLevelStdout();

    opencog::combo::vertex_set ignore_ops;

    const int effort = 3; // effort allocated for reduction (3 is max)

    combo_tree tr;
    while (cin.good()) {
        cin >> tr;
        if (!cin.good())
            break;
        complete_truth_table tt1(tr);
        //cout << "AR" << endl;
        logical_reduce(effort, tr, ignore_ops);
        //cout << "RA" << endl;
        complete_truth_table tt2(tr, integer_log2(tt1.size()));
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
