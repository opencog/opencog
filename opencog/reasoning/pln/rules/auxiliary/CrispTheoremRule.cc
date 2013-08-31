/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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
#include <opencog/util/macros.h>
#include "../../PLN.h"

#include "../Rule.h"
#include "../Rules.h"
#include "../../AtomSpaceWrapper.h"
#include "../../PLNatom.h"
#include "../../BackInferenceTreeNode.h"

using std::vector;
using std::map;
using std::set;

namespace opencog { namespace pln {

CrispTheoremRule::CrispTheoremRule(AtomSpaceWrapper *_asw)
  : Rule(_asw, true, true, "CrispTheoremRule")
{
    // SHOULD NOT BE USED FOR FORWARD INFERENCE!
    // ... but why not? (Joel)
    // Possibly it only works with an actual target, not a generic one --JaredW
    // Note that this input filter is completely wrong.
    inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)ATOM))));
}

map<vtree, vector<vtree> ,less_vtree> CrispTheoremRule::thms;
BBvtree bind_vtree(vtree &targ, map<pHandle, vtree>& binds)
{
    BBvtree thm_substed(new BoundVTree(targ));

    bool changes;
    do
    {               
        changes = false;

        cprintf(4,"Next change...\n");

        for(vtree::pre_order_iterator vit = thm_substed->begin(); vit != thm_substed->end(); vit++)
        {
            cprintf(4,"Next change check...\n");

            pHandle *ph = boost::get<pHandle>(&*vit);
            if (ph)
            {
                cprintf(4,"(ph)");

                map<pHandle, vtree>::iterator it = binds.find(*ph);
                if (it != binds.end()) {

                    cprintf(4,"replacing...[%u]\n", _v2h(*vit));
                    cprintf(4,"with...\n");
                    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
                    printer.print(it->second.begin(),4);

                    thm_substed->replace(vit, it->second.begin());

                    cprintf(4,"replace ok\n");

                    changes = true;
                    goto break_inner;
                    //vit = thm_substed->end(); //break inner loop                            
                }
            }
            else
                cprintf(4,"NOT (ph)");
        }
break_inner:                    
        cprintf(4,"1 change run ok");
    } while (changes);

    return thm_substed;
}

Rule::setOfMPs CrispTheoremRule::o2iMetaExtra(meta outh, bool& overrideInputFilter) const
{
    set<MPs> ret;
    
/*  for (map<vtree, vector<vtree> ,less_vtree>::iterator thm=thms.begin(); thm != thms.end(); thm++)
    {
    rawPrint(thm->first
    }*/

    for (map<vtree, vector<vtree> ,less_vtree>::iterator thm=thms.begin(); thm != thms.end(); thm++)
    {
        map<pHandle, vtree> binds;
        set<pHandle> vars;
        
        /// Grab the FW_VARs
        
        long count1=0, count2=0;
        
        //vector<vtree> thms2(thms->second);
        //vtree thms1(thms->first);
        
        foreach(vtree& targ, thm->second)
        {
			cprintf(1, "Producing thm: checking...\n");
            
            vtree tmp(thm->first);

			NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
            printer.print(tmp.begin(),1);

            //foreach(Vertex v, targ)
            for(vtree::iterator v  = targ.begin(); v != targ.end(); v++)
                if (asw->getType(_v2h(*v)) == FW_VARIABLE_NODE)
                {
                    vars.insert(_v2h(*v));
                    count1++;
                }
        }
        //foreach(Vertex v, thm->first)
        for(vtree::iterator v  = thm->first.begin(); v != thm->first.end(); v++)
            if (asw->getType(_v2h(*v)) == FW_VARIABLE_NODE)
            {
                vars.insert(_v2h(*v));   
                count1++;
            }
                        
        /// Rename the vars
        
        bindingsT newPreBinds;
        
        foreach(pHandle h, vars)
            newPreBinds[h] = _v2h(CreateVar(asw));
                
        cprintf(4,"Bindings are:\n");
                
        foreach(hpair hp, newPreBinds)
			cprintf(4, "%s => %s\n", asw->getName(hp.first).c_str(),
                    asw->getName(hp.second).c_str());
        cprintf(4,"<<< Bindings\n");
                
        /// Replace the old vars with renamed counterparts
                
        vtree           thm_target(thm->first);
        vector<vtree>   thm_args(thm->second);
                
        foreach(vtree& targ, thm_args)
            //foreach(Vertex& v, targ)
            for(vtree::iterator v  = targ.begin(); v != targ.end(); v++)
            {
                bindingsT::iterator bit = newPreBinds.find(_v2h(*v));
                if (bit != newPreBinds.end())
                {
                    *v = Vertex(bit->second);
                    count2++;
                }
            }

        //foreach(Vertex& v, thm_target)
        for(vtree::iterator v  = thm_target.begin(); v != thm_target.end(); v++)
        {
            bindingsT::iterator bit = newPreBinds.find(_v2h(*v));
            if (bit != newPreBinds.end())
            {
                *v = Vertex(bit->second);
                count2++;
            }
        }
        cprintf(4,"From,to\n");
            
        vtree tmp2(thm->first);

        NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
        printer.print(tmp2.begin(),4);
        tmp2 = thm_target;
        printer.print(tmp2.begin(),4);
        
        cprintf(4,"counts: %ld %ld\n", count1, count2);
        
        cprintf(4,"Unifies to %u:\n",_v2h(*outh->begin()));
        
        printer.print(outh->begin(), 4);

/// haxx:: warning!
/// should have different binds for left and right!!!

        if (unifiesTo(asw, thm_target, *outh, binds, binds, true))
        {
            cprintf(4,"Unifies!\n");
            vtree tmp(thm_target);

            printer.print(tmp.begin(), 4);
            
            for (map<pHandle, vtree>::iterator ii=binds.begin(); ii!=binds.end(); ii++)
            {
                cprintf(4,"%u=>\n", ii->first);
                printer.print(ii->second.begin(), 4);
            }

 
            MPs new_thm_args;

            foreach(vtree targ, thm_args)
            {
                cprintf(4,"Subst next...\n");
                
                BBvtree thm_substed = bind_vtree(targ, binds);

                cprintf(4,"FOR:\n");

                printer.print(outh->begin(), 4);
                cprintf(4,"thm_substed\n");
                printer.print(thm_substed->begin(), 4);
                
                new_thm_args.push_back(thm_substed);
            }

            new_thm_args.push_back(BBvtree(new BoundVTree(mva((pHandle)HYPOTHETICAL_LINK,
                                                              *outh))));

            ret.insert(new_thm_args);
			cprintf(1, "Producing thm found.\n");

            // Now, fix the inputFilter to correspond to the new nr of input args.
            const_cast<MPsIn*>(&inputFilter)->clear();
            for (uint i = 0; i < new_thm_args.size(); i++)
                const_cast<MPsIn*>(&inputFilter)->push_back(meta(
                                                                 new tree<Vertex>(mva(
                                                                                      (pHandle)ATOM))));
        }

    }


    overrideInputFilter = true;
    
    return ret;
}


BoundVertex CrispTheoremRule::compute(const VertexSeq& premiseArray,
                                      pHandle CX,
                                      bool fresh) const
{
    /*vtree res(mva((Handle)IMPLICATION_LINK,
        mva(nm->getOutgoing(v2h(premiseArray[0]))[0]),
        mva((Handle)AND_LINK,
            mva(nm->getOutgoing(v2h(premiseArray[0]))[1]),
            mva(nm->getOutgoing(v2h(premiseArray[1]))[1])
        )
    ));*/

    int real_args = premiseArray.size() - 1; /// @todo why?
    
    //vtree res(mva(nm->getOutgoing(v2h(premiseArray[1]))[1]));
    
    cprintf(3,"CrispTheoremRule::compute... [%u]\n", asw->getOutgoing(_v2h(premiseArray[real_args]))[0]);
    
/*  printTree(nm->getOutgoing(v2h(premiseArray[0]))[0],0,0);
    printTree(nm->getOutgoing(v2h(premiseArray[1]))[0],0,0);
    printTree(nm->getOutgoing(v2h(premiseArray[2]))[0],0,0);
    
    cprintf(0,"<<<\n");*/
    
    /// Unravel the HYPOTHETICAL_LINK:
    
    vtree res(make_vtree(asw->getOutgoing(_v2h(premiseArray[real_args]))[0]));
    
//  cprintf(0,"CrispTheoremRule::compute... make_vtree ok\n");
        
    /*for (int i=0;i<premiseArray.size();i++)
        res.append_child(res.begin(), premiseArray[i]*/
    
    TVSeq tvs(real_args);

    for(int i = 0; i < real_args; i++)
    {
        tvs[i] = asw->getTV(_v2h(premiseArray[i]));
    }
    
    bool use_And_rule = (real_args>1);
    
//  cprintf(0,"CrispTheoremRule::compute... TV ok\n");
    
    TruthValue* tv = (use_And_rule?
                      SymmetricAndFormula().compute(tvs)
                      : tvs[0]->clone());
    pHandle ret_h = asw->addAtom(res, *tv, fresh);
    delete tv;
    
	cprintf(3,"CrispTheoremRule::compute produced:\n");

    NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
    printer.print(ret_h);
    
    return Vertex(ret_h);
}

}} // namespace opencog { namespace pln {
