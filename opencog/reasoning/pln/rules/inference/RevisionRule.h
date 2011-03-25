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

#ifndef REVISIONRULE_H
#define REVISIONRULE_H

#include "../../formulas/Formulas.h"

namespace opencog { namespace pln {

class RevisionRule : public GenericRule<RevisionFormula>
{
public:
	RevisionRule(AtomSpaceWrapper *_asw)
	: GenericRule<RevisionFormula>(_asw, false)
	{
		inputFilter.push_back(Btr<atom>(new atom(ANY, 0)));
	}

	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{
		return Rule::setOfMPs(); //No support (yet)
/*		if (!inheritsType(out, ProductLinkType()))
			return Rule::setOfMPs();
		MPs ret;
		ret.insert(neBoundVertexWithNewType(outh, LinkType));

		return ret;*/
	}

	virtual bool valid(Handle* premiseArray, const int n) const
	{
		a1 = asw->getType(premiseArray[0]);
		a2 = asw->getType(premiseArray[1]);

		return	a1 == a2 &&
				asw->getOutgoing(premiseArray[0]) == asw->getOutgoing(premiseArray[1]);
	}

	virtual atom i2oType(Handle* h, const int n) const
	{
		assert(n==2);

		return atom(h[0]);
	}

protected:
	/// WARNING! THE FOLLOWING LINE MAY PRODUCE AN ODD UNRESOLVED LINK ERROR IN MSVS!
	mutable std::vector<Type> ti;
	mutable Type a1, a2;

	void SortTVs(Handle* premiseArray, const int n, TruthValuePtr** retTVs, int* retn) const
	{
	    for (int i = 0; i < n; i++)
			(*retTVs)[i] = asw->getTV(premiseArray[i]);

		*retn = n;
	}

	virtual std::vector<Type> InputTypes() const
	{
		if (ti.empty())
		{
			ti.push_back(a1);
			ti.push_back(a1);
		}
		
		return ti;
	}
	virtual Type ProductLinkType() const
	{
		return a1;
	}
	virtual std::vector<Handle> ProductLinkSequence(Handle* premiseArray) const
	{
		return asw->getOutgoing(premiseArray[0]);
	}
};

}} // namespace opencog { namespace pln {
#endif // REVISIONRULE_H
