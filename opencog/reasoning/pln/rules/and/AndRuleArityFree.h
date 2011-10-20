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

#ifndef ANDRULEARITYFREERULE_H
#define ANDRULEARITYFREERULE_H

#include "../../formulas/Formulas.h"
#include "../Rule.h"

namespace opencog { namespace pln {

/**
	@class ArityFreeAndRule
	Shouldn't be used directly. Use AndRule<number of arguments> instead.
*/

class ArityFreeAndRule : public Rule
{
protected:
    ArityFreeAndRule(AtomSpaceWrapper *_asw)
	: Rule(_asw,true,true,"")
    {}
    SymmetricAndFormula fN;
    AsymmetricAndFormula f2;
public:
    bool validate2(MPs& args) const { return true; }
    
    bool asymmetric(Handle* A, Handle* B) const;
    //Handle compute(Handle A, Handle B, Handle CX = NULL)  const; //std::vector<Handle> vh)
    
    BoundVertex computeSymmetric(const VertexSeq& premiseArray,
                                 pHandle CX = PHANDLE_UNDEFINED,
                                 bool fresh = true) const;

    /**
     * partition premiseArray into atoms of type AND_LINK and others.
     * AND_LINK atoms are inserted in AndLinks and the others are inserted
     * in nodes. Prior insertion neither AndLinks not nodes are emptied.
     *
     * @param premiseArray The sequence of premises
     * @param AndLinks will contain the set of premises of type AND_LINK
     * @param nodes will contain the set of premises of any type but AND_LINK
     */
    void DistinguishNodes(const VertexSeq& premiseArray,
                          std::set<pHandle>& Andlinks,
                          std::set<pHandle>& nodes) const;
    
    BoundVertex compute(const VertexSeq& premiseArray,
                        pHandle CX = PHANDLE_UNDEFINED,
                        bool fresh = true) const=0;
};

}} // namespace opencog { namespace pln {
#endif // ANDRULEARITYFREERULE_H

