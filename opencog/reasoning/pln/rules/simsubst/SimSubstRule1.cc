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

#if 0
Rule::setOfMPs SimSubstRule1::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
/** For simplicity (but sacrificing applicability),
FW_VARs cannot be replaced with children structs.
Links are assumed not inheritable either.
    */
    
    if (outh->begin().number_of_children() != 2
        ||  inheritsType(nm->getType(v2h(*outh->begin())), FW_VARIABLE_NODE))
        return Rule::setOfMPs();

/*  puts("X1");
    rawPrint(*outh,0,0);
    puts("X2");*/
    
    Rule::setOfMPs ret;


//  set<atom> child_nodes;
//  find_child_nodes(outh, child_nodes);
    
    Vertex child = CreateVar(destTable);
    
    for(tree<Vertex>::pre_order_iterator i = outh->begin(); i != outh->end(); i++)
//  for (set<atom>::iterator i = child_nodes.begin(); i != child_nodes.end(); i++)
    {       
/*      puts("-");
        printAtomTree(outh,0,0);
*/
        Vertex old_i = *i;
        *i = child;
        BBvtree templated_atom1(new BoundVTree(*outh));
        *i = old_i;     
        
        BBvtree inhPattern1(new BoundVTree(mva((Handle)INHERITANCE_LINK,
            mva(child), mva(*i))));
        
        MPs ret1;
        ret1.push_back(inhPattern1);
        ret1.push_back(templated_atom1);
        
/*      puts("X");
        rawPrint(*outh,0,0);
        puts("-");
        rawPrint(*templated_atom1,0,0);
        puts("-");
        rawPrint(*inhPattern1,0,0);
        puts("-");*/
        
        overrideInputFilter = true;
        
        ret.insert(ret1);
    }
    
    
    return ret;
}

meta SimSubstRule1::i2oType(const vector<Vertex>& h) const
{
    AtomSpaceWrapper *nm = GET_ATW;

    Handle h0 = v2h(h[0]);
    Handle h1 = v2h(h[1]);
    
    const int N = h.size();
    assert(2==N);
    assert(nm->getType(h0) == INHERITANCE_LINK);
    
    // ( any, Inh(a,b) )
    

    atom ret(h1);
    
    //assert(ret.hs[1].real == nm->getOutgoing(h[1])[0]);

    vector<Handle> hs = nm->getOutgoing(h0);

    /// subst hs[0] to hs[1] (child => parent):
    ret.hs[0]->substitute(atom(hs[1]), atom(hs[0]));

//printAtomTree(ret,0,0);
    
/*  meta ret(new Tree<Vertex>(mva(nm->getType(v2h(h[0])),
        mva(nm->getOutgoing(v2h(h[1]))[0]),
        mva(nm->getOutgoing(v2h(h[0]))[1])));*/
    
	return BBvtree(new BoundVTree(ret.makeHandletree(destTable)));
}
#endif

}} // namespace opencog { namespace pln {
