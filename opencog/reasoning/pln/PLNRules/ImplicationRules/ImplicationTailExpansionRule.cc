#include <platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomTableWrapper.h"
#include "../../Ptlatom.h"
#include "../../BackInferenceTreeNode.h"

namespace reasoning
{

ImplicationTailExpansionRule::ImplicationTailExpansionRule(reasoning::iAtomTableWrapper *_destTable)
: Rule(_destTable, false, true, "ImplicationTailExpansionRule")
{
    inputFilter.push_back(meta(
        new tree<Vertex>(mva((Handle)
            IMPLICATION_LINK,
                mva((Handle)ATOM),
                mva((Handle)ATOM)
        ))
    ));
    inputFilter.push_back(meta(
        new tree<Vertex>(mva((Handle)
            IMPLICATION_LINK,
                mva((Handle)ATOM),
                mva((Handle)ATOM)
        ))
    ));
}

Rule::setOfMPs ImplicationTailExpansionRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    AtomSpace *nm = CogServer::getAtomSpace();
    if (!inheritsType(nm->getType(v2h(*outh->begin())), IMPLICATION_LINK))
        return Rule::setOfMPs();
        
    tree<Vertex>::sibling_iterator hs1 = outh->begin(outh->begin());
    tree<Vertex>::sibling_iterator hs0 = hs1++;
    tree<Vertex>::sibling_iterator hs11 = outh->begin(hs1);
    tree<Vertex>::sibling_iterator hs10 =hs11++;

cprintf(0,"T:%d\n", (Type)(int)v2h(*hs1));
    
    if (hs0  == outh->end(outh->begin()) ||
        hs11 == outh->end(hs1) ||
        !inheritsType(nm->getType(v2h(*hs1)), AND_LINK) )
        return Rule::setOfMPs();

    /// A => (B&C)
    
    MPs ret;

    ret.push_back(BBvtree(new BoundVTree(mva((Handle)IMPLICATION_LINK, 
            mva(*hs0),
            vtree(hs10)))));
    ret.push_back(BBvtree(new BoundVTree(mva((Handle)IMPLICATION_LINK, 
            mva(*hs0),
            vtree(hs11)))));
//      ret.push_back(BBvtree(new BoundVTree(myvar)));

    overrideInputFilter = true;
    
    return makeSingletonSet(ret);
}

BoundVertex ImplicationTailExpansionRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
{
    AtomSpace *nm = CogServer::getAtomSpace();

/*  for (int i=0;i<premiseArray.size();i++)
        printTree(v2h(premiseArray[i]),0,0);
    getc(stdin);getc(stdin);*/
    
    vtree res(mva((Handle)IMPLICATION_LINK,
        mva(nm->getOutgoing(v2h(premiseArray[0]))[0]),
        mva((Handle)AND_LINK,
            mva(nm->getOutgoing(v2h(premiseArray[0]))[1]),
            mva(nm->getOutgoing(v2h(premiseArray[1]))[1])
        )
    ));
        
    /*for (int i=0;i<premiseArray.size();i++)
        res.append_child(res.begin(), premiseArray[i]*/
    
    return Vertex(destTable->addAtom(res, getTruthValue(v2h(SimpleANDRule<2>(destTable).compute(premiseArray,CX).value)),true,true));
}

} // namespace reasoning
