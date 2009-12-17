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

#ifndef ANDPARTITIONRULE_H
#define ANDPARTITIONRULE_H

namespace opencog { namespace pln {

/** @class ANDPartitionRule
	Partitions argument into smaller ANDLinks
*/

class ANDPartitionRule : public Rule
{
	SymmetricANDFormula fN;

public:
	ANDPartitionRule(AtomSpaceWrapper *_asw)
	: Rule(_asw,true,true,"ANDPartitionRule")
	{ }
	
	bool validate2(MPs& args) const { return true; }
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	BoundVertex compute(const std::vector<Vertex>& premiseArray,
                            pHandle CX = PHANDLE_UNDEFINED,
                            bool fresh = true) const;
	
	NO_DIRECT_PRODUCTION;
};

}} // namespace opencog { namespace pln {
#endif // ANDPARTITIONRULE_H
