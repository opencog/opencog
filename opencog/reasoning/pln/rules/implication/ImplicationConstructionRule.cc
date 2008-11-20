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
Rule::setOfMPs ImplicationConstructionRule::o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const
{
        if (!inheritsType(outh.T, IMPLICATION_LINK) || outh.hs.size() != 2)
            return Rule::setOfMPs();

        boost::shared_ptr<MPs> ret(new MPs);

        ret->push_back(boost::shared_ptr<atom>(new atom(outh)));
        (*ret)[0]->T = AND_LINK;

        overrideInputFilter = true;
        
        return makeSingletonSet(ret);
}

atom ImplicationConstructionRule::i2oType(Handle* h, const int n) const
{
        assert(1==n);

        return  atom(IMPLICATION_LINK, 2,
                        new atom(child(h[0], 0)),
                        new atom(child(h[0], 1))
                );
}
#endif

} // namespace reasoning
