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

#ifndef IMPCONSTRUCTIONRULE_H
#define IMPCONSTRUCTIONRULE_H

namespace opencog { namespace pln {
{

// Produces (Implication A B) given (And A B)

#if 0
class ImplicationConstructionRule : public GenericRule<Implication>
{
public:
	ImplicationConstructionRule(AtomSpaceWrapper *_asw)
	: GenericRule<Implication>(_asw, false, "ImplicationConstructionRule")
	{ }

	setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	virtual meta i2oType(const VertexSeq& h) const;
	bool validate2(MPs& args) const { return true; }
};
#endif

} // namespace opencog { namespace pln {
#endif // IMPCONSTRUCTIONRULE_H
