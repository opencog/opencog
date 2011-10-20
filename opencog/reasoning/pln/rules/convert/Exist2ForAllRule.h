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

#ifndef EXIST2FORALLRULE_H
#define EXIST2FORALLRULE_H

namespace opencog { namespace pln {

class Exist2ForAllRule : public Rule
{
    Exist2ForAllRule(AtomSpaceWrapper *_asw)
        : Rule(_asw)
    {
        inputFilter.push_back(new atom(__INSTANCEOF_N, ExistLink));
    }
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        MPs ret;
        ret.insert(new atom(NotLink,
                            neBoundVertexWithNewType(outh, AndLink)));
        return ret;
    }
    
    virtual atom i2oType(Handle* h, const int n) const
    {
        assert(n==1);
        return atomWithNewType(h[0], FORALL_LINK, asw);
    }
    virtual bool valid(Handle* h, const int n) const
    {
        assert(n==1);
        return isSubType(h[0], EXISTS_LINK);
    }
    
    BoundVertex compute(const VertexSeq& premiseArray,
                        Handle CX = NULL, bool fresh = true) const
    {
        assert(n==1);
        
        return Exist2ForAllLink(premiseArray[0]);
    }
}

}} // namespace opencog { namespace pln {
#endif // EXIST2FORALLRULE_H
