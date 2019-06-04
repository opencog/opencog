/*
 * opencog/atoms/truthvalue/AttentionValue.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by Tony Lofthouse <tony_lofthouse@btinternet.com>
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

#include <opencog/attentionbank/types/atom_types.h>
#include <opencog/atoms/value/ValueFactory.h>
#include <opencog/util/exceptions.h>
#include "AttentionValue.h"

using namespace opencog;

const AttentionValue::sti_t AttentionValue::DEFAULTATOMSTI = 0;
const AttentionValue::lti_t AttentionValue::DEFAULTATOMLTI = 0;
const AttentionValue::vlti_t AttentionValue::DEFAULTATOMVLTI = 0;

AttentionValue::AttentionValue(const std::vector<double>& v) :
	FloatValue(ATTENTION_VALUE)
{
	_value = v;
}

AttentionValue::AttentionValue(sti_t s, lti_t l, vlti_t v) :
	FloatValue(ATTENTION_VALUE)
{
	_value.resize(3);
	_value[STI] = s;
	_value[LTI] = l;

	if (v < 0.0) v = 0.0;  // This is what the unit test wants...
	_value[VLTI] = v;
}

AttentionValue::AttentionValue(const AttentionValue& source) :
	FloatValue(ATTENTION_VALUE)
{
	_value.resize(3);
	_value[STI] = source._value[STI];
	_value[LTI] = source._value[LTI];
	_value[VLTI] = source._value[VLTI];
}

AttentionValue::AttentionValue(const ValuePtr& source) :
	FloatValue(ATTENTION_VALUE)
{
	if (source->get_type() != ATTENTION_VALUE)
		throw RuntimeException(TRACE_INFO,
			"Source must be an AttentionValue");

	AttentionValuePtr ap(AttentionValueCast(source));
	_value.resize(3);
	_value[STI] = ap->value()[STI];
	_value[LTI] = ap->value()[LTI];
	_value[VLTI] = ap->value()[VLTI];
}

AttentionValue::sti_t AttentionValue::getSTI() const
{
	return _value[STI];
}

AttentionValue::lti_t AttentionValue::getLTI() const
{
	return _value[LTI];
}

AttentionValue::vlti_t AttentionValue::getVLTI() const
{
	return _value[VLTI];
}

std::string AttentionValue::to_string(const std::string& indent) const
{
	char buffer[256];
	sprintf(buffer, "(av %f %f %f)", getSTI(), getLTI(), getVLTI());
	return buffer;
}

DEFINE_VALUE_FACTORY(ATTENTION_VALUE,
	createAttentionValue, std::vector<double>)
