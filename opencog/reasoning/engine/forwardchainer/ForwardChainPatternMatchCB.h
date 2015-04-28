/*
 * ForwardChainPatternMatchCB.h
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

#ifndef FORWARDCHAINPATTERNMATCHCB_H_
#define FORWARDCHAINPATTERNMATCHCB_H_

#include "FCMemory.h"
#include <opencog/query/Implicator.h>
#include <opencog/query/DefaultPatternMatchCB.h>
#include <opencog/query/AttentionalFocusCB.h>

namespace opencog
{

class ForwardChainPatternMatchCB: public virtual Implicator,
        public virtual DefaultPatternMatchCB,
        public virtual AttentionalFocusCB {
private:
    AtomSpace* _as;
    FCMemory * _fcmem;
public:
    ForwardChainPatternMatchCB(AtomSpace * as);
    virtual ~ForwardChainPatternMatchCB();
    HandleSeq get_products(void);
    void set_fcmem(FCMemory *fcmem);
    //the follwing callbacks are used for guiding the PM to look only the source list
    virtual bool node_match(const Handle& node1, const Handle& node2);
    virtual bool link_match(const LinkPtr& lpat, const LinkPtr& lsoln);

    /**
     * A callback handler of the Pattern matcher used to store references to new conclusion the source list
     */
    virtual bool grounding(const std::map<Handle, Handle> &var_soln,
                           const std::map<Handle, Handle> &pred_soln);

};

} // ~namespace opencog

#endif /* FORWARDCHAINPATTERNMATCHCB_H_ */
