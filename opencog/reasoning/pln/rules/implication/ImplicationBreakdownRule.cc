#include <platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomTableWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace reasoning
{

ImplicationBreakdownRule::ImplicationBreakdownRule(iAtomTableWrapper *_destTable)
: Rule(_destTable,false,true,"ImplicationBreakdown")
{
		inputFilter.push_back(meta(
                new tree<Vertex>(
                mva((Handle)IMPLICATION_LINK,
                    mva((Handle)ATOM),
                    mva((Handle)ATOM)))
            ));
/*      inputFilter.push_back(meta(
                new tree<Vertex>(       
                    mva((Handle)ATOM))
            ));*/
}

Rule::setOfMPs ImplicationBreakdownRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
        ///haxx:: (restricts internal implications

//      if (inheritsType(nm->getType(v2h(*outh->begin())), IMPLICATION_LINK))
//          return Rule::setOfMPs();

        MPs ret;
        Vertex myvar = CreateVar(destTable);

        // Joel: Wrapped up myvar in vtree to fit functions
        ret.push_back(BBvtree(new BoundVTree(mva((Handle)IMPLICATION_LINK, 
            vtree(myvar),
            *outh))));
//		ret.push_back(meta(new BoundVTree(myvar)));
        
        overrideInputFilter = true;
        
        return makeSingletonSet(ret);
}
	
BoundVertex ImplicationBreakdownRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
{
        assert(validate(premiseArray));

//printTree(premiseArray[0],0,1);

        std::vector<Handle> args = GET_ATW->getOutgoing(v2h(premiseArray[0]));
        Type T = GET_ATW->getType(args[1]);
        std::string pname = GET_ATW->getName(args[1]);

        TruthValue* tvs[] = {
            (TruthValue*) &(GET_ATW->getTV(v2h(premiseArray[0]))),
            (TruthValue*) &(GET_ATW->getTV(args[0])),
            (TruthValue*) &(GET_ATW->getTV(args[1]))
        };
        
        TruthValue* retTV =
            ImplicationBreakdownFormula().compute(tvs, 3);

        std::vector<Handle> new_args = GET_ATW->getOutgoing(args[1]);

        Handle ret=NULL;

        /*if (inheritsType(T, NODE))
            ret = destTable->addNode(T, pname,
                    *retTV,
//                  true);          
                    false);
        else*/
    
        assert (!(inheritsType(T, NODE)));

        ret = destTable->addLink(T, new_args,
                    *retTV,
                    RuleResultFreshness);   

        delete retTV;

NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
printer.print(ret, 1);

        return Vertex(ret);
}

} // namespace reasoning
