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

#ifndef EQUI2IMPRULE_H
#define EQUI2IMPRULE_H

namespace opencog { namespace pln {

class Equi2ImpRule : public Rule
{
    /// "A<=>B" => "AND(A=>B, B=>A)"
    Equi2ImpRule(iAtomSpaceWrapper *_asw)
        : Rule(_asw)
    {
        inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(EQUIVALENCE_LINK))));
    }
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        Btr<MPs> ret(new MPs);
        ret->push_back(Btr<atom>(neBoundVertexWithNewType(outh, EQUIVALENCE_LINK)));
        return makeSingletonSet(ret);
    }
    
    virtual atom i2oType(Handle* h, const int n) const
    {
        assert(1 == n);
        
        return atom(AND_LINK, 2,
                    new atom(IMPLICATION_LINK, 2,
                             new atom(child(h[0], 0)),
                             new atom(child(h[0], 1))),
                    new atom(IMPLICATION_LINK, 2,
                             new atom(child(h[0], 1)),
                             new atom(child(h[0], 0)))
                    );
    }
    virtual bool valid(Handle* h, const int n) const
    {
        assert(n==1);
        
        return isSubType(h[0], EQUIVALENCE_LINK);
    }
    
    BoundVertex compute(const vector<Vertex>& premiseArray,
                        Handle CX = NULL,
                        bool fresh = true) const;
};

}} // namespace opencog { namespace pln {
#endif // EQUI2IMPRULE_H
