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

#ifndef STRICTCRISPUNIONRULE_H
#define STRICTCRISPUNIONRULE_H

namespace opencog { namespace pln {

/// Requires that all subtrees are separately produced; hence requires HypothesisRule.
class StrictCrispUnificationRule : public BaseCrispUnificationRule
{
public:
	StrictCrispUnificationRule(AtomSpaceWrapper *_asw)
        : BaseCrispUnificationRule(_asw) {}
	bool validate2(MPs& args) const { return true; }		
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
};

}} // namespace opencog { namespace pln {
#endif // STRICTCRISPUNIONRULE_H
