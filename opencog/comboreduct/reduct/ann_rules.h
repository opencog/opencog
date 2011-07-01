/*
 * opencog/comboreduct/reduct/contin_rules.h
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
#ifndef _REDUCT_CONTIN_RULES_H
#define _REDUCT_CONTIN_RULES_H

#include <opencog/util/RandGen.h>

#include "reduct.h"

#include "../combo/simple_nn.h"
#include "../combo/vertex.h"
#include "../combo/convert_ann_combo.h"

namespace reduct {

using namespace opencog::combo;

//ann reduction rule
// WARNING: this rule should only be used alone, not combined with meta_rules
struct ann_rule : public crule<ann_rule> {
    ann_rule() : crule<ann_rule>::crule("ann_rule") {}
    void operator()(combo_tree& tr,combo_tree::iterator it) const
    {
        tree_transform trans;
        ann net = trans.decodify_tree(tr);
        net.reduce();
        tr = trans.encode_ann(net);
    }
};

} //~namespace reduct

#endif
