/*
 * ForwardChainerCallBack.h
 *
 * Copyright (C) 2014,2015 Misgana Bayetta
 *
 * Author: Misgana Bayetta <misgana.bayetta@gmail.com>
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

#ifndef FORWARDCHAINERCALLBACK_H_
#define FORWARDCHAINERCALLBACK_H_

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Handle.h>

namespace opencog {

enum target_selection_mode {
    TV_FITNESS_BASED, STI_BASED
};

class Rule;
class FCMemory;
class ForwardChainerCallBack {
private:
    AtomSpace* as_;
public:
    ForwardChainerCallBack(AtomSpace* as) :
            as_(as)
    {
    }
    virtual ~ForwardChainerCallBack()
    {
    }
    /**
     * given a set of rules,choose next rule to be applied based on
     * target matching rule premises,fitness evaluation,weight of rule in the current context...etc
     */
    virtual std::vector<Rule*> choose_rule(FCMemory& fcmem) = 0;
    /**
     * Choose additional premises to the chainer based on fitness
     * evaluation in the focus set.
     * @return a set of Handles chosen as a result of applying fitness
     * criteria with respect to the current target.
     */
    virtual HandleSeq choose_premises(FCMemory& fcmem) = 0;
    /**
     * choose next target from the target list
     * @return a handle to the chosen target from target list
     */
    virtual Handle choose_next_target(FCMemory& fcmem) = 0;
    /**
     * apply chosen rule. the default will wrap a custom PM callback class.
     * i.e invokes _pattern_matcher.
     * @return a set of handles created as a result of applying current choosen rule
     */
    virtual HandleSeq apply_rule(FCMemory& fcmem) = 0;
};

} // ~namespace opencog

#endif /* FORWARDCHAINERCALLBACK_H_ */
