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

#include <algorithm>

namespace opencog { namespace pln {

Rule::setOfMPs SimSubstRule1::o2iMetaExtra(meta outh, bool& overrideInputFilter)
const
{
/** For simplicity (but sacrificing applicability),
FW_VARs cannot be replaced with children structs.
Links are assumed not inheritable either.
    */    
    // Skipping links prevents an assert error in BITNode::tryClone();
    // skipping FW_VARS is required to avoid trying to replace FW_VARS already
    // created by this Rule, and/or a crash. -- JaredW

    // There are two extra restrictions I added to make the BIT search
    // efficient enough:
    // Also skip targets that are InheritanceLinks or similar; whether they
    // are an actual target or the child of another SSR1 BITNode, they can
    // be handled better via Deduction. This is also for inference-control
    // efficiency.

    // Also, only substitute ConceptNodes, not PredicateNodes (a heuristic;
    // Inheritance between other types of Nodes isn't necessarily used)

    pHandle top = _v2h(*outh->begin()); // Top of the target vtree

    if (outh->begin().number_of_children() != 2
        ||  asw->isSubType(top, FW_VARIABLE_NODE)
        ||  asw->isSubType(top, INHERITANCE_LINK)
        ||  asw->isSubType(top, IMPLICATION_LINK))
        return Rule::setOfMPs();

/*  puts("X1");
    rawPrint(*outh,0,0);
    puts("X2");*/
    
    Rule::setOfMPs ret;


//  set<atom> child_nodes;
//  find_child_nodes(outh, child_nodes);
    
    Vertex child = CreateVar(asw);
    
    // The input is: an inheritance from any part of the output atom,
    // and a version of the output atom that has that atom in it.
    for(tree<Vertex>::pre_order_iterator i = outh->begin(); i != outh->end(); i++)
//  for (set<atom>::iterator i = child_nodes.begin(); i != child_nodes.end(); i++)
    {       
/*      puts("-");
        printAtomTree(outh,0,0);
*/
        // if this vertex is a link type or FW_VAR, skip it
        /*if ( asw->isSubType(_v2h(*i), LINK) || asw->isSubType(_v2h(*i),
             FW_VARIABLE_NODE)*/
        // Only substitute ConceptNodes
        if (!asw->isSubType(_v2h(*i), CONCEPT_NODE))
            continue;

        // Put the FW_VAR (child) into the templated atom
        Vertex old_i = *i;
        *i = child;
        BBvtree templated_atom1(new BoundVTree(*outh));
        *i = old_i;     
        
        // Put the FW_VAR into an InheritanceLink
        BBvtree inhPattern1;
        if (generalize)
            inhPattern1 = BBvtree(new BoundVTree(mva((pHandle)INHERITANCE_LINK,
                mva(child), mva(*i))));
        else
            inhPattern1 = BBvtree(new BoundVTree(mva((pHandle)INHERITANCE_LINK,
                mva(*i), mva(child))));
        
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
        
        ForceAllLinksVirtual(inhPattern1);
        ForceAllLinksVirtual(templated_atom1);
        
        overrideInputFilter = true;
        
        ret.insert(ret1);
    }
    
    
    return ret;
}

meta SimSubstRule1::i2oType(const vector<Vertex>& h) const
{
    pHandle h0 = boost::get<pHandle>(h[0]);
    pHandle h1 = boost::get<pHandle>(h[1]);
    
    const int N = h.size();
    assert(2==N);
    assert(asw->getType(h0) == INHERITANCE_LINK);
    
    // ( any, Inh(a,b) )
    
    // Make a vtree out of h1
    meta h1_mp = meta(new vtree(h[1]));
    
    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME, 
              NM_PRINTER_DEFAULT_TRUTH_VALUE_PRECISION, 
              NM_PRINTER_DEFAULT_INDENTATION_TAB_SIZE,
			  3);
    printer.print(h1_mp->begin());
    
    // Make it virtual (to get the structure of the link, not just the handle)
    meta ret = ForceAllLinksVirtual(h1_mp);
    
    printer.print(ret->begin());
    
    vector<pHandle> hs = asw->getOutgoing(h0);
    
    // the InhLink (h[0]) is real when this method is called
    // @todo if the child is a link, it will be real. This is OK.

    /// @todo What if child has a different structure to parent?
    if (generalize)
        std::replace(ret->begin(), ret->end(), Vertex(hs[0]), Vertex(hs[1]));
    else
        std::replace(ret->begin(), ret->end(), Vertex(hs[1]), Vertex(hs[0]));

    printer.print(ret->begin());

    return ret;

#if 0
    atom ret(h1);
    
    //assert(ret.hs[1].real == nm->getOutgoing(h[1])[0]);

    vector<pHandle> hs = asw->getOutgoing(h0);

    /// subst hs[0] to hs[1] (child => parent):
    //ret.hs[0]->substitute(atom(hs[1]), atom(hs[0]));
    ret.substitute(atom(hs[1]), atom(hs[0]));

//printAtomTree(ret,0,0);
    
/*  meta ret(new Tree<Vertex>(mva(nm->getType(boost::get<pHandle>(h[0])),
        mva(nm->getOutgoing(boost::get<pHandle>(h[1]))[0]),
        mva(nm->getOutgoing(boost::get<pHandle>(h[0]))[1])));*/
    
	return BBvtree(new BoundVTree(ret.makeHandletree(asw)));
    // this is the line that crashes the BIT. It should be making a normal vtree out of it (or just using a normal vtree)
#endif
}

}} // namespace opencog { namespace pln {
