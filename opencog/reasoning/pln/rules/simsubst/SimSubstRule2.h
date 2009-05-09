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

#ifndef SIMSUBS2RULE_H
#define SIMSUBS2RULE_H

namespace opencog { namespace pln {

/** RHS stays constant, LHS is substed
 */
class SimSubstRule2 : public GenericRule<InhSubstFormula>
{
public:
	SimSubstRule2()
	{
		name = "SimSubstRule";
//		inputFilter.push_back(Btr<atom>(new atom(__INSTANCEOF_N, 1, new atom(INHERITANCE_LINK,0))));
	}
	setOfMPs o2iMetaExtra(meta, bool& overrideInputFilter) const;

	TruthValue** formatTVarray	(const vector<Vertex>& premiseArray, int* newN) const
	{
		TruthValue** tvs = new SimpleTruthValue*[1];

		assert(N==1);

		tvs[0] = (TruthValue*) &(nm->getTV(v2h(premiseArray[0])));
		tvs[1] = (TruthValue*) &(nm->getTV(v2h(premiseArray[1])));

		return tvs;
	}

	meta i2oType(const vector<Vertex>& h) const
	{
		const int n = h.size();
		Handle h0 = v2h(h[0]);
		Handle h1 = v2h(h[1]);
		
		assert(2==n);
		assert(getType(h1) == INHERITANCE_LINK);

		// ( any, Inh(a,b) )

		atom ret(h0);

		//assert(ret.hs[0].real == nm->getOutgoing(h[1])[0]);

		ret.hs[0] = (TruthValue*) atom(nm->getOutgoing(h1)[0]);
		
		vector<Handle> hs = nm->getOutgoing(h0);
		ret.substitute(atom(hs[1]), atom(hs[0])); //  parent for child
		
		return meta(new Tree<Vertex>(ret.maketree()));		
		
		meta ret(new Tree<Vertex>(mva(getType(v2h(h[0])),
			mva(nm->getOutgoing(v2h(h[1]))[0]),
			mva(nm->getOutgoing(v2h(h[0]))[1])));

		return ret;
	}
};

}} // namespace opencog { namespace pln {
#endif // SIMSUBS2RULE_H
