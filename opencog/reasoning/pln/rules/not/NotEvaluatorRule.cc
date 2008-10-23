#include <opencog/util/platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomTableWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace reasoning
{

NotEvaluatorRule::NotEvaluatorRule(reasoning::iAtomTableWrapper *_destTable)
: GenericRule<reasoning::NotFormula>(_destTable, true, "NotEvaluatorRule")
{
        inputFilter.push_back(meta(
            new tree<Vertex>(mva((Handle)NOT_LINK,
                mva((Handle)ATOM)))
        ));
}

meta NotEvaluatorRule::i2oType(const vector<Vertex>& h) const
{
    assert(1==h.size());
    return meta(new tree<Vertex>(mva((Handle)NOT_LINK,
            tree<Vertex>(h[0])
    )));
}

// Private method    
Rule::setOfMPs NotEvaluatorRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
        if (!GET_ATW->inheritsType(GET_ATW->getType(v2h(*outh->begin())), NOT_LINK))
            return Rule::setOfMPs();

		LOG(-10, "SHOULD NOT BE HERE!");
		getc(stdin);getc(stdin);getc(stdin);
        MPs ret;

        assert(outh->begin().number_of_children() == 1);
		ret.push_back(BBvtree(new BoundVTree(outh->begin(outh->begin())))); //1st child

//      printAtomTree(*(*ret)[0],0,4);

        overrideInputFilter = true;
        return makeSingletonSet(ret);
}

} // namespace reasoning
