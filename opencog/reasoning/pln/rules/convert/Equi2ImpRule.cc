#include <opencog/util/platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomSpaceWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace reasoning
{

#if 0
Handle Equi2ImpRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
{
    const int n = premiseArray.size();
    assert(n==1);
    
    pair<Handle, Handle> p = Equi2ImpLink(premiseArray[0]);
    
    return Join<AND_LINK>(p.first, p.second);
}
#endif


} // namespace reasoning
