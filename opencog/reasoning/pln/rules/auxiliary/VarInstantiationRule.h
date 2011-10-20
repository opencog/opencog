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

#ifndef VARINSTANTIATIONRULE_H
#define VARINSTANTIATIONRULE_H

namespace opencog { namespace pln {

template<typename DeductionRuleType, Type InheritanceLinkType>
class VarInstantiationRule :	public Sim2InhRule<MEMBER_LINK>, //public Eval2MemRule,
								public SubsetEvalRule<CONCEPT_NODE>,
								public DeductionRule<DeductionRuleType,MEMBER_LINK>, public Inh2EvalRule
{
/*protected:
	Sim2InhFormula formula;*/
public:
    VarInstantiationRule(AtomSpaceWrapper *_asw)
	: Rule(_asw)
    {
        inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(EVALUATION_LINK))));
        inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(VARIABLE_NODE))));
        inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(CONCEPT_NODE))));
        inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(SIMILARITY_LINK))));
        inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(EVALUATION_LINK))));
    }
    virtual atom i2oType(Handle* h, const int n) const
    {
        return atom(EVALUATION_LINK, 2, new atom(h[0]), new atom(h[2]));
    }

public:
    Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
    {
        return Rule::setOfMPs(); //No support (yet)
        /*		if (!inheritsType(out, ProductLinkType()))
			return Rule::setOfMPs();
                        MPs ret;
                        ret.insert(new atom(atomWithNewType(outh, LinkType)));
                        return ret;*/
    }

    BoundVertex compute(const VertexSeq& premiseArray, Handle CX = NULL,
                        bool fresh = true) const
    {
	assert(n == 5);
	LINKTYPE_ASSERT(premiseArray[0], EVALUATION_LINK);
	LINKTYPE_ASSERT(premiseArray[1], VARIABLE_NODE);
	LINKTYPE_ASSERT(premiseArray[2], CONCEPT_NODE);
	LINKTYPE_ASSERT(premiseArray[3], SIMILARITY_LINK);
	LINKTYPE_ASSERT(premiseArray[4], EVALUATION_LINK);

	HandleSeq PforB = asw->getOutgoing(premiseArray[4]);
	Handle hP = PforB[0];
        
	Handle hC = SatisfyingSet(hP);
	Handle hA = (premiseArray[2]);
	Handle hB = (asw->getOutgoing(premiseArray[3])[1]); //2nd of sim(A,B) std::vector
        
	Handle inhAB = Sim2InhRule<InheritanceLinkType>::compute(&premiseArray[3], 1);
	//Handle inhBP = Eval2MemRule::compute(&premiseArray[0], 1);
	Handle BsP [] = { hB, hC };
	Handle inhBP = SubsetEvalRule::compute(BsP, 2);

	Handle hAB = (inhAB);
	Handle hBC = (inhBP);

	Handle hargs[5];
	hargs[0] = hAB; hargs[1] = hBC; hargs[2] = hA; hargs[3] = hB; hargs[4] = hC;

	Handle inhAP = DeductionRule<DeductionRuleType>::compute(hargs, 5);

	Handle eval1 = Inh2Eval::compute(&inhAP, 1); //The predicate must still be added

	HandleSeq retlist;
	retlist.push_back(premiseArray[0]); //P
	retlist.push_back(hA);

	TruthValuePtr retTV(asw->getTV(eval1));

	Handle ret = asw->addLink(EVALUATION_LINK, retlist, retTV, fresh);	

	//! @todo Delete the resulting dummy 'eval1' node!

	return ret;
    }
};

}} // namespace opencog { namespace pln {
#endif // VARINSTANTIATIONRULE_H
