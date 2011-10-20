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

#ifndef LINK2LINKRULE_H
#define LINK2LINKRULE_H

#include "../GenericRule.h"
#include "../RuleFunctions.h"
#include "../../utils/NMPrinter.h"
#include "../../formulas/Formulas.h"

namespace opencog { namespace pln {

template<typename FormulaType>
class Link2LinkRule : public GenericRule<FormulaType>
{
    typedef GenericRule<FormulaType> super;

    Type SRC_LINK;
    Type DEST_LINK;

    // @todo I don't understand why it is need but without that it does not compile
    AtomSpaceWrapper* asw;

public:
    bool validate2(Rule::MPs& args) const { return true; }
    Link2LinkRule(AtomSpaceWrapper *_asw, Type src, Type dest)
        : super(_asw, false,""),
          SRC_LINK(src), DEST_LINK(dest),
          asw(_asw) // @todo I don't understand why it is needed, asw is already defined in Rule...

    {
        super::name = "Link2Link(" + std::string(Type2Name(SRC_LINK)) + "=>" +
            std::string(Type2Name(DEST_LINK)) +")";
        super::inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)SRC_LINK,
                                                               mva((pHandle)ATOM),
                                                               mva((pHandle)ATOM))
                                                           )));
    }
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        if (!asw->isSubType(_v2h(*outh->begin()), DEST_LINK))
            return Rule::setOfMPs();
        
        Rule::MPs ret;
        
        BBvtree ret_m(new BoundVTree(*outh));
        *ret_m->begin() = Vertex((pHandle)SRC_LINK);
        ret.push_back(ret_m);
        
        overrideInputFilter = true;
        
        return makeSingletonSet(ret);
    }
    
    virtual TVSeq formatTVarray(const VertexSeq& premiseArray) const
    {
        OC_ASSERT(premiseArray.size()==1);
        return TVSeq(1, asw->getTV(_v2h(premiseArray[0])));
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

typedef  Link2LinkRule<Mem2InhFormula> Mem2InhRule;
typedef  Link2LinkRule<Int2ExtFormula> Int2ExtRule;
typedef  Link2LinkRule<Ext2IntFormula> Ext2IntRule;
typedef  Link2LinkRule<Inh2ImpFormula> Inh2ImpRule;
typedef  Link2LinkRule<Imp2InhFormula> Imp2InhRule;
typedef  Link2LinkRule<Mem2EvalFormula> Mem2EvalRule;

}} // namespace opencog { namespace pln {
#endif // LINK2LINKRULE_H
