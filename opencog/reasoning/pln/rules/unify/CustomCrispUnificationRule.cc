#include <opencog/util/platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomTableWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace haxx
{
    /// \todo This data must persist even if the BITNodeRoot is deleted.
    extern map<Handle,vector<Handle> > inferred_from;
    extern map<Handle,reasoning::Rule*> inferred_with;
}

namespace reasoning
{

bool UnificationRuleResultFreshness = true; //false;

Btr<set<BoundVertex > > CustomCrispUnificationRule::attemptDirectProduction(meta outh)
{
    if (GET_ATW->inheritsType(GET_ATW->getType(v2h(*outh->begin())), FORALL_LINK) ||
        GET_ATW->inheritsType(GET_ATW->getType(v2h(*outh->begin())), FW_VARIABLE_NODE))
        return Btr<set<BoundVertex > >();

#if 0
    rawPrint(*outh, outh->begin(),0);
#else 
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    printer.print(vtree(outh->begin()));
#endif
cprintf(3,"FindMatchingUniversals...\n");
    Btr<ModifiedBoundVTree> i = FindMatchingUniversal(outh, ForallLink, destTable);
cprintf(3,"FindMatchingUniversals OK!\n");
    if (!i)
        return Btr<set<BoundVertex > >();

    Btr<set<BoundVertex > > ret(new set<BoundVertex >);
    
    MPs ret1;
    typedef pair<Handle,vtree> phvt;
    DeclareBtr(bindingsT, pre_binds);

    foreach(phvt vp, *i->bindings)
    {
        (*pre_binds)[vp.first] = make_real(vp.second);
#if 0    
        printTree(vp.first,0,0);
        cprintf(0,"=");
        printTree((*pre_binds)[vp.first],0,0);
#else 
        printer.print(vp.first);
        cprintf(0,"=");
        printer.print((*pre_binds)[vp.first]);
#endif
    }

    BBvtree rootAtom(new BoundVTree(*i, pre_binds));
    bind_Bvtree(rootAtom, *i->bindings);
    Handle topologicalStub = destTable->addAtom(*rootAtom, TruthValue::TRIVIAL_TV(), false, true);

    Handle ret_h = destTable->addLink(  GET_ATW->getType(topologicalStub),
                                GET_ATW->getOutgoing(topologicalStub),
                                GET_ATW->getTV(i->original_handle),
                                UnificationRuleResultFreshness);    
    
    ret->insert(BoundVertex(ret_h, pre_binds));

/*  haxx::bitnoderoot->inferred_with[ret_h] = (Rule*)(int)this;
    if (haxx::bitnoderoot->inferred_from[ret_h].empty()) //comes here often, we want the link only once
        haxx::bitnoderoot->inferred_from[ret_h].push_back(ForallLink);
*/
    haxx::inferred_with[ret_h] = (Rule*)this;
    if (haxx::inferred_from[ret_h].empty()) //comes here often, we want the link only once
        haxx::inferred_from[ret_h].push_back(ForallLink);

    return ret;
}

} // namespace reasoning
