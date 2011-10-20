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

#include <opencog/util/platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomSpaceWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace opencog { namespace pln {

ImplicationBreakdownRule::ImplicationBreakdownRule(AtomSpaceWrapper *_asw)
  : Rule(_asw,false,true,"ImplicationBreakdown")
{
    inputFilter.push_back(meta(
                               new tree<Vertex>(
                                                mva((pHandle)IMPLICATION_LINK,
                                                    mva((pHandle)ATOM),
                                                    mva((pHandle)ATOM)))
                               ));
    /*      inputFilter.push_back(meta(
                new tree<Vertex>(       
                    mva((Handle)ATOM))
            ));*/
}

Rule::setOfMPs ImplicationBreakdownRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    ///haxx:: (restricts internal implications

    //      if (asw->isSubType(v2h(*outh->begin()), IMPLICATION_LINK))
    //          return Rule::setOfMPs();

    MPs ret;
    Vertex myvar = CreateVar(asw);

    // Joel: Wrapped up myvar in vtree to fit functions
    ret.push_back(BBvtree(new BoundVTree(mva((pHandle)IMPLICATION_LINK, 
                                             vtree(myvar),
                                             *outh))));
    //		ret.push_back(meta(new BoundVTree(myvar)));
    
    overrideInputFilter = true;
        
    return makeSingletonSet(ret);
}
	
BoundVertex ImplicationBreakdownRule::compute(const VertexSeq& premiseArray, pHandle CX, bool fresh) const
{
    assert(validate(premiseArray));

//printTree(premiseArray[0],0,1);

    std::vector<pHandle> args = asw->getOutgoing(_v2h(premiseArray[0]));
    Type T = asw->getType(args[1]);
    std::string pname = asw->getName(args[1]);
    
    TruthValuePtr tvs[] = {
        asw->getTV(_v2h(premiseArray[0])),
        asw->getTV(args[0]),
        asw->getTV(args[1])
    };
    
    TruthValue* retTV =
        ImplicationBreakdownFormula().compute(TVSeq(tvs, tvs+3));
    
    std::vector<pHandle> new_args = asw->getOutgoing(args[1]);
    
    pHandle ret=PHANDLE_UNDEFINED;
    
    /*if (inheritsType(T, NODE))
      ret = asw->addNode(T, pname,
      *retTV,
      //                  true);          
      false);
      else*/
    
    assert (!(asw->inheritsType(T, NODE)));
    
    ret = asw->addLink(T, new_args, *retTV, fresh);
    
    delete retTV;
    
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    printer.print(ret, 1);
    
    return Vertex(ret);
}

}} // namespace opencog { namespace pln {
