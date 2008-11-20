#include <opencog/util/platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomSpaceWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace reasoning
{

Rule::setOfMPs ANDPartitionRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
        return PartitionRule_o2iMetaExtra(outh, overrideInputFilter, AND_LINK);
}

BoundVertex ANDPartitionRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
{
        const int N = (int)premiseArray.size();
        Handle *hs = new Handle[N];
        
        transform(premiseArray.begin(), premiseArray.end(), &hs[0], GetHandle()); //mem_fun(
//          Concat<DropVertexBindings, GetHandle, BoundVertex, Handle>());

        BoundVertex ret = Vertex(UnorderedCcompute(destTable, AND_LINK, fN, hs,N,CX));
        delete[] hs;
        return ret;
}

} // namespace reasoning
