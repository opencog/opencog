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

#ifndef CUSTOMCRISPUNIONCOMPOSERRULE_H
#define CUSTOMCRISPUNIONCOMPOSERRULE_H

#include "BaseCrispUnificationRule.h"

namespace opencog { namespace pln {

class CustomCrispUnificationRuleComposer : public BaseCrispUnificationRule
{
	pHandle ForallLink;
public:

	CustomCrispUnificationRuleComposer(pHandle _ForallLink,
                                       AtomSpaceWrapper *_asw)
        : BaseCrispUnificationRule(_asw), ForallLink(_ForallLink) {}

	setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;

	/**
		Arg #1: ForAll formula
		Arg #2: TopologicalLink of the desired result
		Args #3-N: each of the atoms in the outgoingset of the ForAll's parent link.
		These args won't be used in forward computation, but they may cause substitutions
		which affect the Arg #2!
	*/

//	BoundVertex compute(const VertexSeq& premiseArray, Handle CX = NULL) const;
	bool validate2(MPs& args) const { return true; }

	NO_DIRECT_PRODUCTION;
};

}} // namespace opencog { namespace pln {
#endif // CUSTOMCRISPUNIONCOMPOSERRULE_H
