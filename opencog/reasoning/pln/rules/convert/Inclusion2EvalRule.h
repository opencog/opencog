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

#ifndef INC2EVALRULE_H
#define INC2EVALRULE_H

namespace opencog { namespace pln {

template<Type LinkType>
class Inclusion2EvalRule : public GenericRule<IdentityFormula>
{
public:
    Inclusion2EvalRule(AtomSpaceWrapper *_asw)
	: GenericRule<IdentityFormula>(_asw,false)
    {
        inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(LinkType))));
    }
    
protected:
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        return Rule::setOfMPs();
        
        Btr<MPs> ret(new MPs);
        ret->push_back(Btr<atom>(neBoundVertexWithNewType(outh, LinkType)));
        return makeSingletonSet(ret);
    }
    
    virtual atom i2oType(Handle* h, const int n) const
    {
        return atom(EVALUATION_LINK, 2,
                    new atom(PREDICATE_NODE, "The ?"),
                    new atom(child(h[0], 0)));
        
        //atomWithNewType(h[0], EVALUATION_LINK);
    }
    
    /// WARNING! THE FOLLOWING LINE MAY PRODUCE AN ODD UNRESOLVED LINK ERROR IN MSVS!
    mutable std::vector<Type> ti;
    
    virtual std::vector<Type> InputTypes() const
    {
        Type L = LinkType;
        
        if (ti.empty())
            {
                ti.push_back(L);
            }
        
        return ti;
    }
    virtual Type ProductLinkType() const
    {
        return EVALUATION_LINK;
    }
    virtual std::vector<Handle> ProductLinkSequence(Handle* premiseArray) const
    {
        // Create P:
        Handle  p = addNode(PREDICATE_NODE, "The ?",
                            TruthValue::TRUE_TV(),
                            false);
        
        HandleSeq hs = asw->getOutgoing(premiseArray[0]);
        
        std::vector<Handle> psat;
        psat.push_back(p);
        psat.push_back(hs[1]); //S
        
        // Create "EvaluationLink P S" <=> "S = SatisfyingSet P"
        asw->addLink(EVALUATION_LINK, psat, TruthValue::TRUE_TV(), fresh);	

        std::vector<Handle> psat2;
        psat2.push_back(p);
        psat2.push_back(hs[0]); //A
        
        return psat2;
    }
};

typedef Inclusion2EvalRule<MEMBER_LINK> Mem2EvalRule;
typedef Inclusion2EvalRule<SUBSET_LINK> Inh2EvalRule;

}} // namespace opencog { namespace pln {
#endif // INC2EVALRULE_H
