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

#ifndef EVAL2MEMRULE_H
#define EVAL2MEMRULE_H

namespace opencog { namespace pln {

class Eval2MemRule : public GenericRule<IdentityFormula>
{
public:
    Eval2MemRule(AtomSpaceWrapper *_asw)
	: GenericRule<IdentityFormula>(_asw, false)
    {
        inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(EVALUATION_LINK))));
    }
    virtual bool valid(Handle* premiseArray, const int n) const
    {
        return isSubType(premiseArray[0], EVALUATION_LINK);
    }
    virtual atom i2oType(Handle* h, const int n) const
    {
        return atom(INHERITANCE_LINK, 2,
                    new atom(child(h[0],1)),
                    new atom(CONCEPT_NODE, "")
                    );
        //return //atomWithNewType(h, MEMBER_LINK);
    }

protected:

    /// WARNING! THE FOLLOWING LINE MAY PRODUCE AN ODD UNRESOLVED LINK ERROR IN MSVS!
    mutable std::vector<Type> ti;
    
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        if (!inheritsType(out, ProductLinkType()))
            return Rule::setOfMPs();
        
        Btr<MPs> ret(new MPs);
        ret->push_back(Btr<atom>(neBoundVertexWithNewType(outh, LinkType)));
        return makeSingletonSet(ret);
    }

    virtual std::vector<Type> InputTypes() const
    {
        if (ti.empty())
            {
                ti.push_back(EVALUATION_LINK);
            }
        
        return ti;
    }
    virtual Type ProductLinkType() const
    {
        return INHERITANCE_LINK;
    }
    virtual std::vector<Handle> ProductLinkSequence(Handle* premiseArray) const
    {
        HandleSeq hs;
        hs.push_back(premiseArray[1]); //A
        hs.push_back(satisfyingSet(premiseArray[0])); //S
        
        return hs;
        
        //return destTable->getHandle(MEMBER_LINK, hs);
    }
};

}} // namespace opencog { namespace pln {
#endif // EVAL2MEMRULE_H
