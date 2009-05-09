/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by Singularity Institute for Artificial Intelligence
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <opencog/util/platform.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomSpaceWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

namespace opencog { namespace pln {

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

}} // namespace opencog { namespace pln {
