#include <opencog/util/platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomTableWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace reasoning
{

#if 0
atom UnorderedLinkPermutationRule::i2oType(Handle* h) const
{
        assert(2==n);
        return  atom(child(h[1],0)); //Child of the topological link gives the topology
}

Rule::setOfMPs UnorderedLinkPermutationRule::o2iMetaExtra(const atom& outh, bool& overrideInputFilter) const
{
        if (!outh.matchType(AND_LINK) || outh.hs.size() > MAX_ARITY_FOR_PERMUTATION)
            return Rule::setOfMPs();

printAtomTree(outh,0,1);

        set< vector<atom> > *ps = newCreatePermutations<atom>(outh.hs);

        boost::shared_ptr<MPs> ret(new MPs);

        boost::shared_ptr<MetaPredicate> newa( new atom(OR_LINK, 0) );

        for (set< vector<atom> >::iterator i = ps->begin(); i != ps->end(); i++)
            newa->hs.push_back(atom(outh.T, *i));

        ret->push_back(newa);

        printAtomTree(*(*ret)[0],0,4);

        /// Add the pseudo atom which determines the TOPOLOGY of the desired result

        boost::shared_ptr<atom> pseudoAtom(new atom(HYPOTHETICAL_LINK, 1, new atom(outh)));
        Handle h = pseudoAtom->attach(destTable);
        ret->push_back(pseudoAtom);

            TruthValue *tv = getTruthValue(h);


        //printAtomTree(*(*ret)[1],0,4);

        delete ps;

        overrideInputFilter = true;

        return makeSingletonSet(ret);
    }
#endif

} // namespace reasoning
