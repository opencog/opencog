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

#ifndef INH2SIMRULE_H
#define INH2SIMRULE_H

namespace opencog { namespace pln {

template<Type InclusionLink>
class Inh2SimRule : public GenericRule<Inh2SimFormula>
{
protected:
    //	mutable std::vector<Type> ti;

    virtual Type ProductLinkType() const
    {
        return SIMILARITY_LINK;
    }
    virtual std::vector<Handle> ProductLinkSequence(Handle* premiseArray) const
    {
        std::vector<Handle> ret;
        ret.push_back(premiseArray[0]);
        ret.push_back(premiseArray[1]);
        
        return ret;
    }
    
public:
    Inh2SimRule(AtomSpaceWrapper *_asw)
	: GenericRule<Inh2SimFormula>(_asw, false)
    {
        inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(INHERITANCE_LINK))));
    }
    virtual bool valid(Handle* premiseArray, const int n) const
    {
        return (linkInherits(InclusionLink, IMPLICATION_LINK)
                || linkInherits(InclusionLink,INHERITANCE_LINK))
            && (asw->getOutgoing(premiseArray[0],0) == asw->getOutgoingng(premiseArray[1],1)
                && asw->getOutgoing(premiseArray[0],1) == asw->getOutgoing(premiseArray[1],0));
    }
    
    virtual atom i2oType(Handle* h, const int n) const
    {
        assert(1==n);
        
        return atomWithNewType(h[0], SIMILARITY_LINK, asw);
    }
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        Btr<MPs> ret(new MPs);
        ret->push_back(Btr<atom>(new atomWithNewType(outh, INHERITANCE_LINK,
                                                     asw)));
        return makeSingletonSet(ret);
    }
};

}} // namespace opencog { namespace pln {
#endif // INH2SIMRULE_H
