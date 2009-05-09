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

#ifndef ANDRULEARITYFREERULE_H
#define ANDRULEARITYFREERULE_H

namespace opencog { namespace pln {

/**
	@class ArityFreeANDRule
	Shouldn't be used directly. Use AndRule<number of arguments> instead.
*/

class ArityFreeANDRule : public Rule
{
protected:
	ArityFreeANDRule(iAtomSpaceWrapper *_destTable)
	: Rule(_destTable,true,true,"")
	{}
	SymmetricANDFormula fN;
	AsymmetricANDFormula f2;
public:
	bool validate2				(MPs& args) const { return true; }

	bool asymmetric(Handle* A, Handle* B) const;
	//Handle compute(Handle A, Handle B, Handle CX = NULL)  const; //std::vector<Handle> vh)
	BoundVertex computeSymmetric(const vector<Vertex>& premiseArray, pHandle CX = PHANDLE_UNDEFINED) const;
	void DistinguishNodes(const vector<Vertex>& premiseArray, set<pHandle>& ANDlinks, set<pHandle>& nodes) const;

	BoundVertex compute(const vector<Vertex>& premiseArray, pHandle CX = PHANDLE_UNDEFINED) const=0;
};

}} // namespace opencog { namespace pln {
#endif // ANDRULEARITYFREERULE_H

