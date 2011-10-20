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

#ifndef ORRULE_H
#define ORRULE_H

#include "../GenericRule.h"

namespace opencog { namespace pln {

class OrRule : public GenericRule<OrFormula>
{
public:
	OrRule(AtomSpaceWrapper *_asw);
    
//    virtual meta targetTemplate() const;

    virtual setOfMPs fullInputFilter() const;
    
	Rule::setOfMPs o2iMetaExtra(meta outh, bool& overrideInputFilter) const;
	
	NO_DIRECT_PRODUCTION;
	
	virtual TVSeq formatTVarray(const VertexSeq& premiseArray) const;
public:
	bool validate2(MPs& args) const { return true; }

	virtual meta i2oType(const VertexSeq& h) const;
};

}} // namespace opencog { namespace pln {
#endif // ORRULE_H
