#include <opencog/util/platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomSpaceWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace reasoning
{

BoundVertex BaseCrispUnificationRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
{
    const int n = premiseArray.size();
    
	cprintf(4, "BaseCrispUnificationRule::compute:");
#if 0    
    for (int i=0;i<n;i++)
        printTree(v2h(premiseArray[i]), 0, 4);
#else 
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    for (int i=0;i<n;i++)
        printer.print(v2h(premiseArray[i]), 4);
#endif
    
    assert(GET_ATW->getType(v2h(premiseArray[0])) == FORALL_LINK);
    assert(GET_ATW->getType(v2h(premiseArray[n-1])) == HYPOTHETICAL_LINK);

    Handle topologicalStub = GET_ATW->getOutgoing(v2h(premiseArray[n-1]))[0];

    const TruthValue& tv = GET_ATW->getTV(v2h(premiseArray[0]));
    
    
    Handle ret= destTable->addLink( GET_ATW->getType(topologicalStub),
                                GET_ATW->getOutgoing(topologicalStub),
                                tv,
                                RuleResultFreshness);   

    return Vertex(ret);
}

} // namespace reasoning
