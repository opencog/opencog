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

#include <opencog/comboreduct/combo/iostream_combo.h>
#include <opencog/comboreduct/combo/similarity.h>

int main(int argc, char** argv)
{
    using namespace std;
    using namespace opencog;
    using namespace opencog::combo;

    if (argc < 2) {
        cout << "Usage :" << endl <<
             argv[0] << " <combo-tree> [combo-tree]" << endl;
        exit(1);
    }

    if (2 == argc) {
        combo_tree tr;
        stringstream ss;
        ss << argv[1];
        ss >> tr;

        cout << "Input tree: " << tr << endl;

        tree_branch_vector btv = tree_flatten(tr);

        foreach(auto pr, btv) {
           cout << "Vect: " << pr.first << " count: " << pr.second << endl;
        }
        exit(0);
    }

    combo_tree tra, trb;
    stringstream ssa, ssb;
    ssa << argv[1];
    ssa >> tra;
    ssb << argv[2];
    ssb >> trb;

    size_t dist = tree_similarity(tra, trb);
    cout << "diff: " << dist << endl;
    cout << "tree a: " << tra << endl;
    cout << "tree b: " << trb << endl;

    return 0;
}
