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

#ifndef SCHOLEMPRODRULE_H
#define SCHOLEMPRODRULE_H

namespace reasoning
{

/** @class ScholemFunctionProductionRule
	
*/

class ScholemFunctionProductionRule : public Rule
{
protected:
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	BoundVertex compute(const vector<Vertex>& premiseArray, pHandle CX = PHANDLE_UNDEFINED) const
	{
		assert(0);

		return Vertex(PHANDLE_UNDEFINED);
	}
public:
	ScholemFunctionProductionRule(iAtomSpaceWrapper *_destTable)
	: Rule(_destTable,false,false,"ScholemFunctionProductionRule")
	{
		//inputFilter.push_back(new atom(result));
	}
	bool validate2				(MPs& args) const { return true; }

	Btr<set<BoundVertex > > attemptDirectProduction(meta outh);
};

} // namespace reasoning
#endif // SCHOLEMPRODRULE_H
