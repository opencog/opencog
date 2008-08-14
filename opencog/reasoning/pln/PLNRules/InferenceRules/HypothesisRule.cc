#include <platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomTableWrapper.h"
#include "../../Ptlatom.h"
#include "../../BackInferenceTreeNode.h"

#define HYPRULE_MAKES_ZERO_CONFIDENCE_ATOMS false

namespace reasoning
{

Handle _v2h(const Vertex& v) { return v2h(v); }

boost::shared_ptr<set<BoundVertex> > HypothesisRule::attemptDirectProduction(meta outh)
{
    AtomSpace *nm = CogServer::getAtomSpace();
    set<BoundVertex>* ret = new set<BoundVertex>;
    
    Type t = nm->getTypeV(*outh);
    bool hyp_link = inheritsType(t, HYPOTHETICAL_LINK);

	if (HYPRULE_MAKES_ZERO_CONFIDENCE_ATOMS || hyp_link)
        if (!hasFW_VAR(*outh))
        {
 			 cprintf(4,"HYP0:\n");
             NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
             printer.print(outh->begin(), 4);

            ret->insert(BoundVertex(destTable->addAtom(*outh, TruthValue::TRIVIAL_TV(), false, true)));

			 cprintf(4,"HYP:\n");
             printer.print(v2h(ret->begin()->value), 4);
		}

    return boost::shared_ptr<set<BoundVertex> >(ret);
}

Rule::setOfMPs HypothesisRule:: o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    assert(0);
    return Rule::setOfMPs();
}

} // namespace reasoning
