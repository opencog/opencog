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

#ifndef EQUI2IMPRULE_H
#define EQUI2IMPRULE_H

namespace opencog { namespace pln {

class Equi2ImpRule : public GenericRule<IdentityFormula>
{
public:
    /// "A<=>B" => "And(A=>B, B=>A)"
    Equi2ImpRule(AtomSpaceWrapper *_asw)
        : GenericRule<IdentityFormula>(_asw, false, "Equi2ImpRule")
    {
        //inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(EQUIVALENCE_LINK))));
        inputFilter.push_back(meta(new vtree(
                mva((pHandle)EQUIVALENCE_LINK,
                    mva((pHandle)ATOM),
                    mva((pHandle)ATOM))
                )));

    }
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        if (!asw->isSubType(_v2h(*outh->begin()), AND_LINK))
            return Rule::setOfMPs();

        Rule::MPs ret;

        BBvtree ret_m(new BoundVTree(*outh));
        *ret_m->begin() = Vertex((pHandle)EQUIVALENCE_LINK);
        ret.push_back(ret_m);

        overrideInputFilter = true;

        return makeSingletonSet(ret);
    }
    
    meta i2oType(const VertexSeq& hs) const
    {
        assert(1==hs.size());

        Vertex A = asw->getOutgoing(_v2h(hs[0]))[0];
        Vertex B = asw->getOutgoing(_v2h(hs[0]))[1];
        
        return meta(new vtree(mva((pHandle)AND_LINK,
                        mva((pHandle)IMPLICATION_LINK,
                                vtree(A),
                                vtree(B)),
                        mva((pHandle)IMPLICATION_LINK,
                                vtree(B),
                                vtree(A))
                                )));
    }
    
    TVSeq formatTVarray(const VertexSeq& premiseArray) const
    {
        OC_ASSERT(premiseArray.size()==1);
        return TVSeq(1, asw->getTV(_v2h(premiseArray[0])));
    }

    bool validate2(Rule::MPs& args) const { return true; }
};

}} // namespace opencog { namespace pln {
#endif // EQUI2IMPRULE_H
