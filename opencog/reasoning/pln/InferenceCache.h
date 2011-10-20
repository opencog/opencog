/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
 * All Rights Reserved
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
#ifndef OPENCOG_INFERENCE_CACHE_H
#define OPENCOG_INFERENCE_CACHE_H

#include "rules/RuleProvider.h"

class BITNodeUTest;

namespace opencog {

#ifdef USE_BITUBIGRAPHER
class BITUbigrapher;
#endif

namespace pln {

class BITNode;
class BITNodeRoot;

class InferenceCache
{
protected:
    friend class BITNodeRoot;
    friend class BITNode;
    friend class ::BITNodeUTest;

public:

    // temporary haxx

    std::set<Btr<BITNodeRoot> > roots;

protected:

    typedef std::set<BITNode*> BITNodeCacheT;

    BITNodeCacheT nodes;

    // Statistics

    std::map<pHandle,BITNode*> hsource;
    long inferenceNodes; // counts the number of nodes in the BIT

    // A map of which FWVariableNode is owned by which BITNode(s).
    // Cloning a BITNode when a result is found for one argument would result in
    // remaining FWVars being owned by both BITNodes.
    std::map<Vertex, std::set<BITNode*> > varOwner;

    // Not currently used.
    std::map<Rule*, float> priority;

    /** Records which BITNodes use one another.
     * List is maintained in order to know which BITNodes directly
     * or indirectly use (ie. Have as a subtree, with possibly some
     * inheritance-bindings) BITNode b. This information is used (only) to
     * prevent circularity: a BITNode cannot indirectly or directly use itself.
     */
    std::map<BITNode*, std::set<BITNode*> > users;

    std::set<BITNode*> BITNodeTemplates;

    // A whole bunch of methods to access them appropriately. For mining or BIT or statistical purposes.
    // Constructor?
public:
    ~InferenceCache();
    InferenceCache() { }

    static InferenceCache* standardInferenceCache();
};

} } // opencog::pln

#endif // OPENCOG_INFERENCE_CACHE_H
