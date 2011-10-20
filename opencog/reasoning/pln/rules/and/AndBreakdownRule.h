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

#ifndef ANDBREAKDOWNRULE_H
#define ANDBREAKDOWNRULE_H

#include "../Rule.h"
#include "../../formulas/Formulas.h"

namespace opencog { namespace pln {

/// Produces A given an AndLink containing A (and N other Atoms)
/// You want multiple copies of this Rule, for different N.
class AndBreakdownRule : public Rule
{
protected:
    AndBreakdownFormula formula;
    const uint N;
    
public:
    AndBreakdownRule(AtomSpaceWrapper *_asw, int _N)
	: Rule(_asw,true,true,""), N(_N)
    {
        name = "AndBreakdownRule/" + i2str(N);
	
        inputFilter.push_back(meta(
                                   new tree<Vertex>(mva((pHandle)AND_LINK,
                                                        mva((pHandle)ATOM),
                                                        mva((pHandle)ATOM)))));
        inputFilter.push_back(meta(
                                   new tree<Vertex>(mva((pHandle)HYPOTHETICAL_LINK,
                                                        mva((pHandle)ATOM),
                                                        mva((pHandle)ATOM)))));
    }
    
    bool validate2(MPs& args) const { return true; }
    
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
    
    NO_DIRECT_PRODUCTION;
    
    BoundVertex compute(const VertexSeq& premiseArray,
                        pHandle CX = PHANDLE_UNDEFINED,
                        bool fresh = true) const;
};

}} // namespace opencog { namespace pln {
#endif // ANDBREAKDOWNRULE_H

