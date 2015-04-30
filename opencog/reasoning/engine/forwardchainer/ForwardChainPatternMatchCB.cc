/*
 * ForwardChainPatternMatchCB.cc
 *
 * Copyright (C) 2014 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>  Sept 2014
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

#include "ForwardChainPatternMatchCB.h"

using namespace opencog;

ForwardChainPatternMatchCB::ForwardChainPatternMatchCB(AtomSpace * as) :
        Implicator(as), DefaultPatternMatchCB(as), AttentionalFocusCB(as),
                _as(as)
{
    _fcmem = nullptr;
}

ForwardChainPatternMatchCB::~ForwardChainPatternMatchCB()
{
}

bool ForwardChainPatternMatchCB::node_match(const Handle& node1,
                                            const Handle& node2)
{
    //constrain search within premise list
    bool ret = _fcmem->isin_premise_list(node2)
            and (_fcmem->is_search_in_af() ?
                    AttentionalFocusCB::node_match(node1, node2) :
                    DefaultPatternMatchCB::node_match(node1, node2));

    cout << "Node " << node2->toString() << "RET = " << ret << endl;
    return ret;

}

bool ForwardChainPatternMatchCB::link_match(const LinkPtr& lpat,
                                            const LinkPtr& lsoln)
{
    //constrain search within premise list
    bool ret = _fcmem->isin_premise_list(Handle(lsoln))
            and (_fcmem->is_search_in_af() ?
                    AttentionalFocusCB::link_match(lpat, lsoln) :
                    DefaultPatternMatchCB::link_match(lpat, lsoln));

    cout << "Link " << ((Handle) lsoln)->toString() << "RET = " << ret << endl;
    return ret;
}

bool ForwardChainPatternMatchCB::grounding(
        const std::map<Handle, Handle> &var_soln,
        const std::map<Handle, Handle> &pred_soln)
{
    cout << "[DEBUG] Grounding " << endl;

    Handle h = inst.instantiate(implicand, var_soln);
    if (Handle::UNDEFINED != h) {
        cout << "[DEBUG] adding grounding " << h->toString() << endl;
        result_list.push_back(h);
    }
    return true;
}

void ForwardChainPatternMatchCB::set_fcmem(FCMemory *fcmem)
{
    _fcmem = fcmem;
}
HandleSeq ForwardChainPatternMatchCB::get_products()
{
    auto product = result_list;
    result_list.clear();
    return product;
}
