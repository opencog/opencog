#include <opencog/util/platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomTableWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace reasoning
{

Rule::setOfMPs CustomCrispUnificationRuleComposer::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    AtomTableWrapper *nm = GET_ATW;
    if (inheritsType(nm->getType(v2h(*outh->begin())), FORALL_LINK) ||
        inheritsType(nm->getType(v2h(*outh->begin())), FW_VARIABLE_NODE))
        return Rule::setOfMPs();

#if 0
    rawPrint(*outh, outh->begin(),0);
#else 
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    printer.print(vtree(outh->begin()));
#endif

    Btr<ModifiedBoundVTree> i = FindMatchingUniversal(outh, ForallLink, destTable);

    if (!i)
        return Rule::setOfMPs();

    setOfMPs ret;
    
    MPs ret1;
    typedef pair<Handle,vtree> phvt;
    DeclareBtr(bindingsT, pre_binds);

    foreach(phvt vp, *i->bindings)
        (*pre_binds)[vp.first] = make_real(vp.second);

    ret1.push_back(BBvtree(new BoundVTree(mva(i->original_handle), pre_binds)));

    BBvtree rootAtom(new BoundVTree(mva((Handle)HYPOTHETICAL_LINK, *i), pre_binds));

    ret1.push_back(rootAtom);

    for_each(ret1.begin(),
                ret1.end(),
                boost::bind(&bind_Bvtree, _1, *i->bindings));

    ret.insert(ret1);

    overrideInputFilter = true;
    
cprintf(3,"Crispu.o2i: OK! Solution vector size=%u\n", (uint) ret.size());
    
    return ret;
}

} // namespace reasoning
