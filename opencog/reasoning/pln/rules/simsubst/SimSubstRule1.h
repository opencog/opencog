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

#ifndef SIMSUBS1RULE_H
#define SIMSUBS1RULE_H

namespace opencog { namespace pln {

/**
 * Left side stays constant, RHS is substed.
 
 
1) Inh A B
2) C (any atom that contains/is A)

Output:
C but with B instead of A.

 */
class SimSubstRule1 : public GenericRule<InhSubstFormula>
{
protected:
    /** Whether to generalize, i.e. go in the specific->general direction.
    If false, it goes in the specific->general direction (i.e. the result contains
    A instead of B.
    */
    bool generalize;
public:
	SimSubstRule1(AtomSpaceWrapper *_asw, bool _generalize = true)
	: GenericRule<InhSubstFormula>(_asw, false, "SimSubstRule"),
	  generalize(_generalize)
	{
		inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)INHERITANCE_LINK,
			mva((pHandle)ATOM),
			mva((pHandle)ATOM)))));
		inputFilter.push_back(meta(new tree<Vertex>(mva((pHandle)ATOM))));
	}
	setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	TruthValue** formatTVarray	(const vector<Vertex>& premiseArray, int* newN) const
	{
	    AtomSpaceWrapper *nm = GET_ASW;
	
		TruthValue** tvs = new TruthValue*[1];

		const int N = (int)premiseArray.size();
		assert(N==2);

		tvs[0] = (TruthValue*) &(nm->getTV(boost::get<pHandle>(premiseArray[0])));
		tvs[1] = (TruthValue*) &(nm->getTV(boost::get<pHandle>(premiseArray[1])));

		return tvs;
	}

	bool validate2(MPs& args) const { return true; }

	virtual meta i2oType(const vector<Vertex>& h) const;
};

}} // namespace opencog { namespace pln {
#endif // SIMSUBS1RULE_H
