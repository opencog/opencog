/*
 * opencog/ubigraph/BITUbigrapher.h
 *
 * Copyright (C) 2008-2009 by OpenCog Foundation
 * All Rights Reserved
 *
 * Written by Jared Wigmore <jared.wigmore@gmail.com>
 * Some code adapted from Ubigrapher
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

#ifndef _OPENCOG_BITUBIGRAPHER
#define _OPENCOG_BITUBIGRAPHER

// NOTE: USE_BITUBIGRAPHER is defined (or not) in BackInferenceTreeNode.h, not here
// (So that this file can include that one, not the other way round.)

// (haxx::) currently makes a BITNodeRoot automatically use a BITUbigrapher.
// It should have an option for attaching it to a particular BIT
// (or at least be automatically used by any BITNodeRoot, but be disabled by
// default, as now).

//#ifdef USE_BITUBIGRAPHER

#include <opencog/atomspace/AtomSpace.h>

#include <opencog/reasoning/pln/AtomSpaceWrapper.h> // BackInferenceTreeNode.h breaks otherwise
#include <opencog/reasoning/pln/BackInferenceTreeNode.h>
//#include "Ubigrapher.h"

#ifdef USE_BITUBIGRAPHER

#include <set>
#include <vector>

using namespace opencog;
using namespace opencog::pln;

namespace opencog
{

/** BITUbigrapher - shows a PLN BackInferenceTree on the Ubigraph visualisation server.
 *
 * \url http://http://ubietylab.net/ubigraph/
 */
class BITUbigrapher// : public Ubigrapher
{
protected:
    //! Initialise the various styles
    virtual void setStyles();

public:
    BITUbigrapher();
    virtual ~BITUbigrapher() {}

    void init();

    bool isConnected();

    bool connected;

//    void drawRoot(BITNode* root);

    void drawBITNodeFitness(int node_id, float fitness);

    void drawBITNodeLabel(BITNode * node, int node_id);

//    void drawBITNode(BITNode* node, std::vector<std::set<ParametrizedBITNode> >
//                     children);
    // ruleNumber: 0 == BITNodeRoot, 1 == root variable scoper, otherwise the index of the Rule in the RuleProvider
    void drawBITNode(BITNode* node, int ruleNumber = 0);

    void drawArgSlots(BITNode* node);

    // reusing_template should be false normally, but true if parent is reusing child via the BITNode template system
    void drawBITLink(int parent_id, int child_id, int slot, bool reusing_template = false);

    int findBITNodeID(BITNode* node);

    // todo: these should have consistent, event-driven-style names.
    // todo: graph() would show it again, which is wrong.
    /// Hide a BITNode (either a Generator or a BITNode which disobeyed pool policy)
    void hideBITNode(BITNode* node);

    void foundResult(BITNode* node);

    void markClone(BITNode* existing, BITNode* clone);

    // indicate that parent reused child.
    void markReuse(BITNode* parent, BITNode* child, int slot);

    //! Redraw the ubigraph, based on how the BIT is now.
    void graph();
};

} // namespace opencog

namespace haxx
{
opencog::BITUbigrapher* BITUSingleton;
}

#endif // USE_BITUBIGRAPHER

#endif // _OPENCOG_BITUBIGRAPHER
