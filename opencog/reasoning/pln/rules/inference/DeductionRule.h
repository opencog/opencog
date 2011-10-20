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

#ifndef DEDUCTIONRULE_H
#define DEDUCTIONRULE_H

#include <opencog/util/iostreamContainer.h>
#include <opencog/util/algorithm.h>
#include "../GenericRule.h"

namespace opencog { namespace pln {

#define CHECK_ARGUMENT_VALIDITY_FOR_DEDUCTION_RULE 0

static const std::string DeductionRuleSuffixStr = "DeductionRule";

//template<typename DeductionFormula, Type InclusionLink> //=IMPLICATION_LINK>
template<typename DeductionFormula>
class DeductionRule : public GenericRule<DeductionFormula>
{
    typedef GenericRule<DeductionFormula> super;

    Type InclusionLink;

    meta i2oType(const VertexSeq& h) const
    {
        OC_ASSERT(h.size()==2);
        pHandle AB = _v2h(h[0]);
        pHandle BC = _v2h(h[1]);
        OC_ASSERT(super::asw->getArity(AB)==2);
        OC_ASSERT(super::asw->getArity(BC)==2);
        pHandle A = super::asw->getOutgoing(AB,0);
        pHandle C = super::asw->getOutgoing(BC,1);
        OC_ASSERT(A != PHANDLE_UNDEFINED);
        OC_ASSERT(C != PHANDLE_UNDEFINED);
        
        return meta(new vtree(mva((pHandle)InclusionLink, mva(A), mva(C))));
    }
    
    bool validate2 (Rule::MPs& args) const
    {
        return (args.size() == 2 && !(*args[0] == *args[1]));
    }
    
    TVSeq formatTVarray(const VertexSeq& premiseArray) const
    {
        TVSeq tvs(5);
        
        OC_ASSERT(premiseArray.size()==2);

        pHandle AB = _v2h(premiseArray[0]);
        pHandle BC = _v2h(premiseArray[1]);

        pHandleSeq nodesAB = super::asw->getOutgoing(AB);
        pHandleSeq nodesBC = super::asw->getOutgoing(BC);
        
        if (CHECK_ARGUMENT_VALIDITY_FOR_DEDUCTION_RULE && !equal(nodesAB[1], nodesBC[0]))
            {
                cprintf(0, "Invalid deduction arguments:\n");
                NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
                printer.print(AB);
                printer.print(BC);
                OC_ASSERT(equal(nodesAB[1], nodesBC[0]));
            }
        
        tvs[0] = super::asw->getTV(AB);
        tvs[1] = super::asw->getTV(BC);
        tvs[2] = super::asw->getTV(nodesAB[0]);
        tvs[3] = super::asw->getTV(nodesAB[1]); //== nodesBC[0]);
        tvs[4] = super::asw->getTV(nodesBC[1]);
        
        return tvs;
    }

public:
    
    DeductionRule(AtomSpaceWrapper *_asw, Type linkType)
        : super(_asw, false, DeductionRuleSuffixStr), InclusionLink(linkType) 
    {
        OC_ASSERT(classserver().isA(linkType,LINK));
        std::string linkName = classserver().getTypeName(linkType);
        super::name = linkName.substr(0,linkName.find("Link")) + super::name;

        //! @todo should use real variable for the other input.
        super::inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)InclusionLink,
                                                               mva((pHandle)ATOM),
                                                               mva((pHandle)ATOM)))));		
        super::inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)InclusionLink,
                                                               mva((pHandle)ATOM),
                                                               mva((pHandle)ATOM)))));		
    }
    
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        if (!super::asw->isSubType(_v2h(*outh->begin()), InclusionLink))
            return Rule::setOfMPs();
        
        Rule::MPs ret;
        
        tree<Vertex>::iterator top0 = outh->begin();
	
        Vertex var = CreateVar(super::asw);
	
        ret.push_back(BBvtree(new BoundVTree(mva((pHandle)InclusionLink,
                                                 vtree(outh->begin(top0)),
                                                 mva(var)))));
        ret.push_back(BBvtree(new BoundVTree(mva((pHandle)InclusionLink,
                                                 mva(var),
                                                 vtree(outh->last_child(top0))))));
        
        overrideInputFilter = true;
        
        return makeSingletonSet(ret);
    }

    meta targetTemplate() const
    {
        // Using FWVars rather than ATOM is potentially less efficient, but
        // required for ForAll unification to match it (CustomCrispUnificationRule)
        return(meta(new vtree(mva((pHandle)InclusionLink, 
                                  vtree(CreateVar(super::asw)),
                                  vtree(CreateVar(super::asw))))));
    }
};

}} // namespace opencog { namespace pln {
#endif // DEDUCTIONRULE_H
