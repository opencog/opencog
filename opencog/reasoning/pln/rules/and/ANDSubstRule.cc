#include <opencog/util/platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomSpaceWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace reasoning
{

/*Rule::setOfMPs ANDSubstRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    if (!inheritsType(nm->getType(v2h(*outh->begin())), IMPLICATION_LINK)
        || outh->number_of_children() != 2)
        return Rule::setOfMPs();        

    tree<Vertex>::sibling_iterator hs1 = outh->begin(outh->begin());
    tree<Vertex>::sibling_iterator hs0 = hs1++;
    
    if (!inheritsType(nm->getType(v2h(*hs0)), AND_LINK)
        || (   !inheritsType(nm->getType(v2h(*hs0)), AND_LINK)
            && !inheritsType(nm->getType(v2h(*hs0)), FW_VARIABLE_NODE)
    )
        return Rule::setOfMPs();
    
}

BoundVertex ANDSubstRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
{
} */

} // namespace reasoning
