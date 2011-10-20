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

#ifndef INH2EVALRULE_H
#define INH2EVALRULE_H

namespace opencog { namespace pln {

class Inh2EvalRule : public GenericRule<IdentityFormula>
{
protected:
    mutable std::vector<Type> ti;

public:
    bool validate2(Rule::MPs& args) const { return true; }
    Inh2EvalRule(AtomSpaceWrapper *_asw)
	: GenericRule<IdentityFormula>(_asw,false,"")
    {
        GenericRule<FormulaType>::name = "Inh2Eval";
        GenericRule<FormulaType>::inputFilter.push_back(meta(
                                                             new tree<Vertex>(
                                                                              mva((Handle)INHERITANCE_LINK,
                                                                                  mva((Handle)ATOM),
                                                                                  mva((Handle)ATOM))
                                                                              )));
    }
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        if (!isSubType(_v2h(*outh->begin()), EVALUATION_LINK))
            return Rule::setOfMPs();
        
        Rule::MPs ret;
        
        BBvtree ret_m(new BoundVTree(*outh));
        *ret_m->begin() = Vertex((Handle)SRC_LINK);
        ret.push_back(ret_m);
	
        overrideInputFilter = true;
        
        return makeSingletonSet(ret);
    }
    
    virtual TVSeq formatTVarray(const VertexSeq& premiseArray) const
    {
        assert(premiseArray.size()==1);
        return TVSeq(1, &(asw->getTV(_v2h(premiseArray[0]))));
    }
    
    virtual meta i2oType(const VertexSeq& h) const
    {
        assert(1==h.size());
        
        meta ret = atomWithNewType(h[0], DEST_LINK, asw);
        cprintf(3,"i2otype() outputs: ");
#if 0
        rawPrint(*ret, ret->begin(), 3);
#else
        NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME, 
                          NM_PRINTER_DEFAULT_TRUTH_VALUE_PRECISION, 
                          NM_PRINTER_DEFAULT_INDENTATION_TAB_SIZE, 
                          3);
        printer.print(ret->begin());
#endif
        
        return ret;
    }
    NO_DIRECT_PRODUCTION;
};

}} // namespace opencog { namespace pln {
#endif // INH2EVALRULE_H
