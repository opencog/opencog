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

#ifndef ANDRULE_H
#define ANDRULE_H

namespace opencog { namespace pln {

class AndRule : public ArityFreeAndRule
{
public:
	AndRule(AtomSpaceWrapper *_asw)
        : ArityFreeAndRule(_asw)
	{
		name = "And Evaluator Rule";
	}
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	//Btr<set<BoundVertex > > attemptDirectProduction(meta outh);

	BoundVertex compute(const VertexSeq& premiseArray,
                            pHandle CX = PHANDLE_UNDEFINED,
                            bool fresh = true) const;
	
	///! Direct production was used here before. @todo Check whether this should be resumed!
	NO_DIRECT_PRODUCTION;
};

}} // namespace opencog { namespace pln {
#endif // ANDRULE_H
