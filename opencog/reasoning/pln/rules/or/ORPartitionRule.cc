#include <opencog/util/platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomTableWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace reasoning
{

Rule::setOfMPs ORPartitionRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
        return PartitionRule_o2iMetaExtra(outh, overrideInputFilter, OR_LINK);
        
/*      if (!inheritsType(nm->getType(v2h(*outh->begin())), OR_LINK) ||
            outh->begin().number_of_children() <= 2)
            return Rule::setOfMPs();

** TODO: Update to BoundVTree. I no longer remember how this was supposed to work!
        MPs ret;

        //vector<atom> hs;
//          hs.push_back(out_hs[0]);

        vector<atom>::iterator bigstart = out_hs.begin();
        vector<atom> hs2(++bigstart, out_hs.end());
//      hs.push_back(atom(OR_LINK, hs2));

        ret.push_back(BBvtree(new BoundVTree(atom(out_hs[0]).maketree()));
        ret.push_back(BBvtree(new BoundVTree(atom(OR_LINK, hs2).maketree()));

        overrideInputFilter = true;
        
        return makeSingletonSet(ret);*/
}

BoundVertex ORPartitionRule::compute(const vector<Vertex>& premiseArray, Handle CX) const
{
/*  Handle *hs = new Handle[premiseArray.size()];
    transform(premiseArray.begin(), premiseArray.end(), hs[0], DropVertexBindings()); //mem_fun(
    const int n = premiseArray.size();*/

    BoundVertex ret = regularOR->compute(premiseArray,CX);
//  delete[] hs;
    return ret;
}

} // namespace reasoning
