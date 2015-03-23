/*
 * opencog/embodiment/Learning/main/post_learning-rewriter.cc
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Nil Geisweiller
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

#include <opencog/comboreduct/combo/vertex.h>
#include <opencog/comboreduct/type_checker/type_tree.h>

#include <opencog/embodiment/Learning/RewritingRules/RewritingRules.h>
#include <opencog/embodiment/AvatarComboVocabulary/AvatarComboVocabulary.h>

#include <iostream>

using namespace std;
using namespace opencog;
using namespace AvatarCombo;
using namespace reduct;

int main()
{

    combo_tree tr;

    while (cin.good()) {
        // cin >> tr;
        AvatarCombo::operator>>(cin, tr);
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

        //print the tree before reduction, for debugging
        cout << "Before : " << tr << endl;

        post_learning_rewrite(tr);

        cout << "After  : " << tr << endl;

    }
}
