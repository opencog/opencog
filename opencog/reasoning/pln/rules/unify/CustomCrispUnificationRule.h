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

#ifndef CUSTOMCRISPUNIONRULE_H
#define CUSTOMCRISPUNIONRULE_H

namespace reasoning
{

class CustomCrispUnificationRule : public Rule
{
protected:
	pHandle ForallLink;
public:

	CustomCrispUnificationRule(pHandle _ForallLink, iAtomSpaceWrapper *_destTable)
	: Rule(_destTable,false,false,"CrispUnificationRule"), ForallLink(_ForallLink)
	{
		inputFilter.push_back(meta(
				new tree<Vertex>(mva(ATOM))));
	}

	BoundVertex compute(const vector<Vertex>& premiseArray, pHandle CX = PHANDLE_UNDEFINED) const
	{
		assert(0);
		return Vertex(PHANDLE_UNDEFINED);
	}

	setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const
	{ return Rule::setOfMPs(); }

	bool validate2(MPs& args) const { return true; }

	Btr<set<BoundVertex > > attemptDirectProduction(meta outh);
};


} // namespace reasoning
#endif // CUSTOMCRISPUNIONRULE_H
