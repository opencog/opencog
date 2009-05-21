/*
 * opencog/ubigraph/BITUbigrapher.h
 *
 * Copyright (C) 2008-2009 by Singularity Institute for Artificial Intelligence
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

#define USE_BITUBIGRAPHER // (haxx) currently makes a BITNodeRoot automatically use a BITUbigrapher

#include <opencog/atomspace/AtomSpace.h>

#include <opencog/reasoning/pln/AtomSpaceWrapper.h> // BackInferenceTreeNode.h breaks otherwise
#include <opencog/reasoning/pln/BackInferenceTreeNode.h>

using namespace opencog;

namespace opencog
{

/** BITUbigrapher - shows a PLN BackInferenceTree on the Ubigraph visualisation server.
 *
 * \url http://http://ubietylab.net/ubigraph/
 */
class BITUbigrapher
{
private:
protected:
    //! Initialise the various styles
    virtual void setStyles();
public:
    BITUbigrapher();
    virtual ~BITUbigrapher() {}

    void drawRoot(BITNode* root);
    
    void drawBITNode(BITNode* node, vector<set<ParametrizedBITNode> > children);
        
    //! Redraw the ubigraph, based on how the BIT is now.
    void graph();
};

} // namespace opencog

namespace haxx {
    opencog::BITUbigrapher* BITUSingleton;
}

#endif // _OPENCOG_BITUBIGRAPHER
