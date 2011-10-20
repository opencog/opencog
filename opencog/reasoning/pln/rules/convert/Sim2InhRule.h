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

#ifndef SIM2INHRULE_H
#define SIM2INHRULE_H

namespace opencog { namespace pln {

/** OutputInheritanceLink:
 * InheritanceLink, IntensionalInheritanceLink, SubsetLink, ImplicationLink,
 *IntensionalImplictionLink, or ExtensionalImplicationLink
 */
template<Type OutputInheritanceLink>
class Sim2InhRule : public Rule
{
    Sim2InhFormula formula;
public:
    Sim2InhRule(AtomSpaceWrapper *_asw)
	
    {
        inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(SIMILARITY_LINK,0))));
    }
    virtual atom i2oType(Handle* h, const int n) const
    {
        assert(n==1);
        return atomWithNewType(h[0], OutputInheritanceLink, asw);
    }
    Rule::setOfMPs o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const
    {
        if (!inheritsType(out, ProductLinkType()))
            return Rule::setOfMPs();
        
        Btr<MPs> ret(new MPs);
        ret->push_back(Btr<atom>(neBoundVertexWithNewType(outh, OutputInheritanceLink)));
        return makeSingletonSet(ret);
    }
    
    BoundVertex compute(const VertexSeq& premiseArray, Handle CX = NULL) const
    {
        assert(n == 1);
        LINKTYPE_ASSERT(premiseArray[0], SIMILARITY_LINK);
	
        CreatePremises;
        
        FirstPLNTruthValue* tvs[20];
        
        tvs[0] = asw->getTV(premiseArray[0]);
        
        TruthValue* retTV = formula.compute(tvs, n);
        
        int i=0;
        
        for (i = 0; i < n; i++)
            delete tvs[i];
        
        HandleSeq sab, sba, retlist;
        
        sab.push_back(premiseArray[0]);
        sab.push_back(premiseArray[1]);
        
        sba.push_back(premiseArray[1]);
        sba.push_back(premiseArray[0]);
        
        Handle  ab = asw->addLink(OutputInheritanceLink, sab,
                                  retTV->clone(),
                                  RuleResultFreshness);	
        
        Handle  ba = asw->addLink(OutputInheritanceLink, sba,
                                  retTV->clone(),
                                  RuleResultFreshness);	
        
        retlist.push_back(ab);
        retlist.push_back(ba);
        
        Handle  ret = asw->addLink(LIST_LINK, retlist,
                                   retTV->clone(),
                                   RuleResultFreshness);			
        
        delete retTV;
        
        return ret;
    }
};

}} // namespace opencog { namespace pln {
#endif // SIM2INHRULE_H
