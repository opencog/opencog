/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
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

#ifndef CUSTOMCRISPUNIONRULE_H
#define CUSTOMCRISPUNIONRULE_H

#include <boost/lexical_cast.hpp>

#include "../Rule.h"

namespace opencog { namespace pln {

class CustomCrispUnificationRule : public Rule
{
protected:
    /** The ForAllLink that this rule has been instantiated for.
     */
    pHandle hForAllLink;

public:
    CustomCrispUnificationRule(pHandle _hForAllLink, AtomSpaceWrapper *_asw)
        : Rule(_asw,false,false,"CustomCrispUnificationRule"),
        hForAllLink(_hForAllLink)
    {
        //append the pHandle of the FOEALL_LINK so that the rule name is unique
        name += boost::lexical_cast<std::string>(hForAllLink); 
        inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)ATOM))));
    }

    BoundVertex compute(const std::vector<Vertex>& premiseArray,
                        pHandle CX = PHANDLE_UNDEFINED,
                        bool fresh = true) const
    {
        // Assumedly, this assert(0) is because a CustomCrispUnification rule
        // only makes sense when the variables within the ForAllLink are bound.
        assert(0);
        return Vertex(PHANDLE_UNDEFINED);
    }

    setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    { return Rule::setOfMPs(); }

    bool validate2(MPs& args) const { return true; }

    Btr<std::set<BoundVertex > > attemptDirectProduction(meta outh,
                                                         bool fresh = true);
};

}} // namespace opencog { namespace pln {
#endif // CUSTOMCRISPUNIONRULE_H
