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

#ifndef DEDUCTIONRULE_H
#define DEDUCTIONRULE_H

namespace opencog { namespace pln {

#define CHECK_ARGUMENT_VALIDITY_FOR_DEDUCTION_RULE 0

//template<typename DeductionFormula, Type InclusionLink> //=IMPLICATION_LINK>
template<typename DeductionFormula>
class DeductionRule : public GenericRule<DeductionFormula>
{
    //DeductionFormula f;
    Type InclusionLink;

    //! @todo I don't understand why it is needed but without this it does not
    //! compile
    AtomSpaceWrapper* asw;

    meta i2oType(const std::vector<Vertex>& h) const
    {
        assert(h.size()==2);

        assert(asw->getArity(boost::get<pHandle>(h[0]))==2);
        assert(asw->getArity(boost::get<pHandle>(h[1]))==2);
        assert(asw->getOutgoing(boost::get<pHandle>(h[0]),0) != PHANDLE_UNDEFINED);
        assert(asw->getOutgoing(boost::get<pHandle>(h[1]),1) != PHANDLE_UNDEFINED);
	
        return meta(new vtree(mva((pHandle)InclusionLink, 
                                         vtree(Vertex(asw->getOutgoing(boost::get<pHandle>(h[0]),0))),
                                         vtree(Vertex(asw->getOutgoing(boost::get<pHandle>(h[1]),1)))
                                         )));
    }
    
    bool validate2 (Rule::MPs& args) const
    {
        return (args.size() == 2 && !(*args[0] == *args[1]));
    }
    
    TruthValue** formatTVarray(const std::vector<Vertex>& premiseArray,
                               int* newN) const
    {
        TruthValue** tvs = (TruthValue**)new SimpleTruthValue*[5];
        
        assert(premiseArray.size()==2);
        
        pHandleSeq nodesAB = GET_ASW->getOutgoing(boost::get<pHandle>(premiseArray[0]));
        pHandleSeq nodesBC = GET_ASW->getOutgoing(boost::get<pHandle>(premiseArray[1]));
        
        if (CHECK_ARGUMENT_VALIDITY_FOR_DEDUCTION_RULE && !equal(nodesAB[1], nodesBC[0]))
            {
                cprintf(0, "Invalid deduction arguments:\n");
                NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
                printer.print(_v2h(premiseArray[0]));
                printer.print(_v2h(premiseArray[1]));
                assert(equal(nodesAB[1], nodesBC[0]));
            }
        
        tvs[0] = (TruthValue*) &(asw->getTV(boost::get<pHandle>(premiseArray[0])));
        tvs[1] = (TruthValue*) &(asw->getTV(boost::get<pHandle>(premiseArray[1])));
        tvs[2] = (TruthValue*) &(asw->getTV(nodesAB[0]));
        tvs[3] = (TruthValue*) &(asw->getTV(nodesAB[1])); //== nodesBC[0]);
        tvs[4] = (TruthValue*) &(asw->getTV(nodesBC[1]));
        
        return tvs;
    }

public:
    
    //DeductionRule(iAtomSpaceWrapper *_asw)
    //: GenericRule<DeductionFormula>(_asw,false,"DeductionRule")
    DeductionRule(AtomSpaceWrapper *_asw, Type linkType)
	: GenericRule<DeductionFormula>(_asw,false,"DeductionRule"),
        InclusionLink(linkType),
        asw(_asw) // @todo I don't understand why it is needed, asw is already defined in Rule...
    {
        //! @todo should use real variable for the other input.
	
        GenericRule<DeductionFormula>::inputFilter.push_back(meta(
                                                                  new tree<Vertex>(
                                                                                   mva((pHandle)InclusionLink,
                                                                                       mva((pHandle)ATOM),
                                                                                       mva((pHandle)ATOM)))
                                                                  ));		
        GenericRule<DeductionFormula>::inputFilter.push_back(meta(
                                                                  new tree<Vertex>(
                                                                                   mva((pHandle)InclusionLink,
                                                                                       mva((pHandle)ATOM),
                                                                                       mva((pHandle)ATOM)))
                                                                  ));		
    }
    
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        if ( !asw->inheritsType((Type)_v2h(*outh->begin()), InclusionLink))
            return Rule::setOfMPs();
        
        Rule::MPs ret;
        
        tree<Vertex>::iterator top0 = outh->begin();
	
        Vertex var = CreateVar(asw);
	
        ret.push_back(BBvtree(new BoundVTree(mva((pHandle)InclusionLink,
                                                 tree<Vertex>(outh->begin(top0)),
                                                 mva(var)))));
        ret.push_back(BBvtree(new BoundVTree(mva((pHandle)InclusionLink,
                                                 mva(var),
                                                 tree<Vertex>(outh->last_child(top0))		
                                                 ))));
        
        overrideInputFilter = true;
        
        return makeSingletonSet(ret);
    }
   
    meta targetTemplate() const
    {
        return(meta(new vtree(mva((pHandle)InclusionLink, 
                                         mva((pHandle)ATOM),
                                         mva((pHandle)ATOM)
                                         ))));
    }
    
    NO_DIRECT_PRODUCTION;
};
        
}} // namespace opencog { namespace pln {
#endif // DEDUCTIONRULE_H
