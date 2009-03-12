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

namespace reasoning
{

#define CHECK_ARGUMENT_VALIDITY_FOR_DEDUCTION_RULE 0

//template<typename DeductionFormula, Type InclusionLink> //=IMPLICATION_LINK>
template<typename DeductionFormula>
class DeductionRule : public GenericRule<DeductionFormula>
{
    //DeductionFormula f;
    Type InclusionLink;

	meta i2oType(const vector<Vertex>& h) const
	{
		assert(h.size()==2);
		AtomSpaceWrapper *nm = GET_ATW;	
		assert(nm->getArity(v2h(h[0]))==2);
		assert(nm->getArity(v2h(h[1]))==2);
		assert(nm->getOutgoing(v2h(h[0]),0) != Handle::UNDEFINED);
		assert(nm->getOutgoing(v2h(h[1]),1) != Handle::UNDEFINED);
	
		return meta(new tree<Vertex>(mva(Handle(InclusionLink), 
						vtree(Vertex(nm->getOutgoing(v2h(h[0]),0))),
						vtree(Vertex(nm->getOutgoing(v2h(h[1]),1)))
				)));
	}
	bool validate2 (Rule::MPs& args) const
	{
		return (args.size() == 2 && !(*args[0] == *args[1]));
	}
	TruthValue** formatTVarray(const vector<Vertex>& premiseArray, int* newN) const
	{
		TruthValue** tvs = (TruthValue**)new SimpleTruthValue*[5];

		assert(premiseArray.size()==2);

		std::vector<Handle> nodesAB = GET_ATW->getOutgoing(v2h(premiseArray[0]));
		std::vector<Handle> nodesBC = GET_ATW->getOutgoing(v2h(premiseArray[1]));

		//assert(equal(nodesAB[1], nodesBC[0]));

		if (CHECK_ARGUMENT_VALIDITY_FOR_DEDUCTION_RULE && !equal(nodesAB[1], nodesBC[0]))
		{
			cprintf(0, "Invalid deduction arguments:\n");
#if 0            
			printTree(v2h(premiseArray[0]),0,0);
			printTree(v2h(premiseArray[1]),0,0);
#else 
            NMPrinter printer(NMP_HANDLE|NMP_TYPE_NAME);
            printer.print(v2h(premiseArray[0]));
            printer.print(v2h(premiseArray[1]));
#endif             
			getc(stdin);getc(stdin);
			return NULL;
		}

		AtomSpaceWrapper *nm = GET_ATW;
		tvs[0] = (TruthValue*) &(nm->getTV(v2h(premiseArray[0])));
		tvs[1] = (TruthValue*) &(nm->getTV(v2h(premiseArray[1])));
		tvs[2] = (TruthValue*) &(nm->getTV(nodesAB[0]));
		tvs[3] = (TruthValue*) &(nm->getTV(nodesAB[1])); //== nodesBC[0]);
		tvs[4] = (TruthValue*) &(nm->getTV(nodesBC[1]));

		return tvs;
	}

public:

	//DeductionRule(iAtomSpaceWrapper *_destTable)
	//: GenericRule<DeductionFormula>(_destTable,false,"DeductionRule")
	DeductionRule(iAtomSpaceWrapper *_destTable, Type linkType)
	: GenericRule<DeductionFormula>(_destTable,false,"DeductionRule"), InclusionLink(linkType) 
	{
		/// TODO: should use real variable for the other input.
		
		GenericRule<DeductionFormula>::inputFilter.push_back(meta(
				new tree<Vertex>(
				mva((Handle)InclusionLink,
					mva((Handle)ATOM),
					mva((Handle)ATOM)))
			));		
		GenericRule<DeductionFormula>::inputFilter.push_back(meta(
				new tree<Vertex>(
				mva((Handle)InclusionLink,
					mva((Handle)ATOM),
					mva((Handle)ATOM)))
			));		
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		if ( !GET_ATW->inheritsType((Type)(int)v2h(*outh->begin()).value(), InclusionLink))
			return Rule::setOfMPs();

		std::string varname = ("$"+GetRandomString(10));

		Rule::MPs ret;

		tree<Vertex>::iterator top0 = outh->begin();
		
		Vertex var = CreateVar(GenericRule<DeductionFormula>::destTable);
		
		ret.push_back(BBvtree(new BoundVTree(mva(Vertex((Handle)InclusionLink),
			tree<Vertex>(outh->begin(top0)),
			mva(var)))));
		ret.push_back(BBvtree(new BoundVTree(mva(Vertex((Handle)InclusionLink),
			mva(var),
			tree<Vertex>(outh->last_child(top0))		
			))));

		overrideInputFilter = true;

		return makeSingletonSet(ret);
	}

	NO_DIRECT_PRODUCTION;
};

} // namespace reasoning
#endif // DEDUCTIONRULE_H
