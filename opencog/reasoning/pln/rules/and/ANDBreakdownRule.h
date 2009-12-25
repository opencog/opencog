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

#ifndef ANDBREAKDOWNRULE_H
#define ANDBREAKDOWNRULE_H

#include "../Rule.h"
#include "../../formulas/Formulas.h"

namespace opencog { namespace pln {

Rule::setOfMPs makeSingletonSet(Rule::MPs& mp);
template<int N>
class ANDBreakdownRule : public Rule
{
protected:
    ANDBreakdownFormula formula;
    
public:
    ANDBreakdownRule(AtomSpaceWrapper *_asw)
	: Rule(_asw,true,true,"")
    {
        name = "ANDBreakdownRule/" + i2str(N);
	
        inputFilter.push_back(meta(
                                   new tree<Vertex>(mva((Handle)AND_LINK,
                                                        mva((Handle)ATOM),
                                                        mva((Handle)ATOM)))));
        inputFilter.push_back(meta(
                                   new tree<Vertex>(mva((Handle)HYPOTHETICAL_LINK,
                                                        mva((Handle)ATOM),
                                                        mva((Handle)ATOM)))));
    }
    
    bool validate2(MPs& args) const { return true; }
    
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        /// And parameters aren't ordered. Therefore, the order in which we feed them
        /// here is irrelavent. But we need a hypothetical parameter that will later reming
        /// which kind of atom we need to produce.
        
        MPs ret;
        BBvtree andlink(new BoundVTree);
        
        andlink->set_head(Vertex((Handle)AND_LINK));
	
        andlink->append_child(andlink->begin(), outh->begin());
	
        for (int i = 1; i < N; i++)
            andlink->append_child(andlink->begin(),
                                  BoundVTree(CreateVar(asw)).begin());
        
        ret.push_back(andlink);
        ret.push_back(BBvtree(new BoundVTree(mva((Handle)HYPOTHETICAL_LINK,*outh))));
	
        overrideInputFilter = true;	
	
        return makeSingletonSet(ret);
    }
    
    NO_DIRECT_PRODUCTION;
    
    BoundVertex compute(const std::vector<Vertex>& premiseArray,
                        Handle CX = NULL,
                        bool fresh = true) const
    {
        std::vector<pHandle> hs = asw->getOutgoing(boost::get<pHandle>(premiseArray[0]));
        
        assert(premiseArray.size() == 2);
        assert(hs.size() == N);
        assert(asw->getArity(boost::get<pHandle>(premiseArray[1])) == 1);	  	  
        
        atom topological_model(asw->getOutgoing(boost::get<pHandle>(premiseArray[1]))[0]);
        
        for (uint i = 0; i < hs.size(); i++)
            if (atom(hs[i]) == topological_model)
                return BoundVertex(hs[i]);
        
	/*LOG(0,"Topo model was:");
          printAtomTree(topological_model,0,0);
          LOG(0,"hs:");
          for (uint i = 0; i < hs.size(); i++)
          printTree(hs[i],0,0);*/
	
	//LOG(0, "ANDBREAKDOWN: NO TOPOLOGICAL MODEL FOUND!");
        assert(0);
        
        return Vertex((Handle)NULL);
	
        /*	std::vector<Handle> hs = asw->getOutgoing(premiseArray[0]);

	assert(premiseArray.size() == 2);
	assert(hs.size() == N);
	assert(asw->getArity(premiseArray[1]) == 1);
	  
	tree<Vertex>::iterator top0 = premiseArray[0].begin();

	tree<Vertex> topological_model(premiseArray[1])[0]
	tree<Vertex>::iterator topological_model = (premiseArray[1])[0];

	for (int i = 0; i < hs.size(); i++)
	for (tree<Vertex>::sibling_iterator i = premiseArray[0].begin();
			i != premiseArray[0].end(); i++)		
		if (*i == topological_model)
			return *i;

	//LOG(0, "ANDBREAKDOWN: NO TOPOLOGICAL MODEL FOUND!");
	assert(0);*/
  }
};

}} // namespace opencog { namespace pln {
#endif // ANDBREAKDOWNRULE_H

