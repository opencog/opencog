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

#ifndef CRISPTHEOREMRULE_H
#define CRISPTHEOREMRULE_H

namespace opencog { namespace pln {

//! A^B^C... implies Z. May contain variables.
//! This Rule was not originally documented, but what it does is exactly the
//! same thing a normal predicate logic chainer does at every step. It appears
//! that, for some reason, the PLN BC can't do this through a combination of
//! other Rules that are already available.
//! It appears to be a tacky temporary Rule, e.g. using FWVars in the input Atoms
//! rather than ForAll+VariableNodes.
class CrispTheoremRule : public Rule
{
public:
	static std::map<vtree, std::vector<vtree> ,less_vtree> thms;
	NO_DIRECT_PRODUCTION;

	CrispTheoremRule(AtomSpaceWrapper *_asw);
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	BoundVertex compute(const VertexSeq& premiseArray,
                        pHandle CX = PHANDLE_UNDEFINED,
                        bool fresh = true) const;

	bool validate2(MPs& args) const { return true; }
};

}} // namespace opencog { namespace pln {
#endif // CRISPTHEOREMRULE_H
