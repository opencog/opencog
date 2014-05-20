/*
 * opencog/comboreduct/main/tree-sim.cc
 *
 * Copyright (C) 2014 Aidyia Limited
 * All Rights Reserved
 *
 * Written by Linas Vepstas
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

#include <opencog/comboreduct/similarity/similarity.h>
#include <opencog/comboreduct/combo/iostream_combo.h>

int main(int argc, char** argv)
{
    using namespace std;
    using namespace opencog;
    using namespace opencog::combo;

    if (argc != 2) {
        cout << "Usage :" << endl <<
             argv[0] << " <combo-tree>" << endl;
        exit(1);
    }

    combo_tree tr;
    stringstream ss;
    ss << argv[1];
    ss >> tr;

    cout << "Input tree: " << tr << endl;

    tree_branch_vector btv = tree_flatten(tr);

    foreach(auto pr, btv) {
       cout << "Vect: " << pr.first << " count: " << pr.second << endl;
    }
}
