#include <platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomTableWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace reasoning
{

/*
Rule::setOfMPs SimSubstRule2::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    //      if (outh.hs.size() != 2 || outh.hs[1].T == FW_VARIABLE_NODE || inheritsType(outh.hs[1].T,LINK))
    return Rule::setOfMPs();
    
    string varname1 = ("$"+GetRandomString(10));
    
    Rule::setOfMPs ret(new set<boost::shared_ptr<MPs> >);
    atom child(FW_VARIABLE_NODE, varname1);
    
    boost::shared_ptr<atom> inhPattern2(new atom(INHERITANCE_LINK, 2,
        new atom(child), new atom(outh.hs[1])));
    
    boost::shared_ptr<MPs> ret2(new MPs);
    atom *templated_atom2 = new atom(outh); //RHS template
    templated_atom2->hs[1] = child;
    ret2->push_back(boost::shared_ptr<atom>(templated_atom2));
    ret2->push_back(inhPattern2);
    
    overrideInputFilter = true;
    
    ret->insert(ret2);
    
    return ret;
}*/

} // namespace reasoning
