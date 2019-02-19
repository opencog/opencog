/*
 * AttentionValueOfLink.cc
 *
 * Copyright (C) 2015, 2018, 2019 Linas Vepstas
 *
 * Author: Linas Vepstas <linasvepstas@gmail.com>  January 2009
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <opencog/atoms/base/Node.h>
#include <opencog/attentionbank/avalue/AttentionValue.h>
#include "AttentionValueOfLink.h"

using namespace opencog;

AttentionValueOfLink::AttentionValueOfLink(const HandleSeq& oset, Type t)
	: ValueOfLink(oset, t)
{
	if (not nameserver().isA(t, ATTENTION_VALUE_OF_LINK))
	{
		const std::string& tname = nameserver().getTypeName(t);
		throw InvalidParamException(TRACE_INFO,
			"Expecting an AttentionValueOfLink, got %s", tname.c_str());
	}
}

AttentionValueOfLink::AttentionValueOfLink(const Link &l)
	: ValueOfLink(l)
{
	// Type must be as expected
	Type tscope = l.get_type();
	if (not nameserver().isA(tscope, ATTENTION_VALUE_OF_LINK))
	{
		const std::string& tname = nameserver().getTypeName(tscope);
		throw InvalidParamException(TRACE_INFO,
			"Expecting an AttentionValueOfLink, got %s", tname.c_str());
	}
}

// ---------------------------------------------------------------

static const Handle& attn_key(void)
{
	static Handle ak(createNode(PREDICATE_NODE, "*-AttentionValueKey-*"));
	return ak;
}

static AttentionValuePtr get_av(const Handle& h)
{
	auto pr = h->getValue(attn_key());
	if (nullptr == pr) return AttentionValue::DEFAULT_AV();
	return AttentionValueCast(pr);
}

// ---------------------------------------------------------------

/// When executed, this will return the AttentionValue
ValuePtr AttentionValueOfLink::execute(AtomSpace* as, bool silent)
{
	size_t ary = _outgoing.size();
	if (1 != ary)
		throw SyntaxException(TRACE_INFO, "Expecting one atom!");

	return ValueCast(get_av(_outgoing[0]));
}

// =============================================================

StiOfLink::StiOfLink(const HandleSeq& oset, Type t)
	: ValueOfLink(oset, t)
{
	if (not nameserver().isA(t, STI_OF_LINK))
	{
		const std::string& tname = nameserver().getTypeName(t);
		throw InvalidParamException(TRACE_INFO,
			"Expecting an StiOfLink, got %s", tname.c_str());
	}
}

StiOfLink::StiOfLink(const Link &l)
	: ValueOfLink(l)
{
	// Type must be as expected
	Type tscope = l.get_type();
	if (not nameserver().isA(tscope, STI_OF_LINK))
	{
		const std::string& tname = nameserver().getTypeName(tscope);
		throw InvalidParamException(TRACE_INFO,
			"Expecting an StiOfLink, got %s", tname.c_str());
	}
}

// ---------------------------------------------------------------

/// When executed, this will return the Strengths of all of the
/// atoms in the outgoing set.
ValuePtr StiOfLink::execute(AtomSpace* as, bool silent)
{
	std::vector<double> strengths;

	for (const Handle& h : _outgoing)
	{
		// Cannot take the attention value of an ungrounded variable.
		Type t = h->get_type();
		if (VARIABLE_NODE == t or GLOB_NODE == t)
			return get_handle();

		strengths.push_back(get_av(h)->getSTI());
	}

	return createFloatValue(strengths);
}

// =============================================================

LtiOfLink::LtiOfLink(const HandleSeq& oset, Type t)
	: ValueOfLink(oset, t)
{
	if (not nameserver().isA(t, LTI_OF_LINK))
	{
		const std::string& tname = nameserver().getTypeName(t);
		throw InvalidParamException(TRACE_INFO,
			"Expecting an LtiOfLink, got %s", tname.c_str());
	}
}

LtiOfLink::LtiOfLink(const Link &l)
	: ValueOfLink(l)
{
	// Type must be as expected
	Type tscope = l.get_type();
	if (not nameserver().isA(tscope, LTI_OF_LINK))
	{
		const std::string& tname = nameserver().getTypeName(tscope);
		throw InvalidParamException(TRACE_INFO,
			"Expecting an LtiOfLink, got %s", tname.c_str());
	}
}

// ---------------------------------------------------------------

/// When executed, this will return the Confidences of all of the
/// atoms in the outgoing set.
ValuePtr LtiOfLink::execute(AtomSpace* as, bool silent)
{
	std::vector<double> confids;

	for (const Handle& h : _outgoing)
	{
		// Cannot take the attention value of an ungrounded variable.
		Type t = h->get_type();
		if (VARIABLE_NODE == t or GLOB_NODE == t)
			return get_handle();

		confids.push_back(get_av(h)->getLTI());
	}

	return createFloatValue(confids);
}

DEFINE_LINK_FACTORY(AttentionValueOfLink, ATTENTION_VALUE_OF_LINK)
DEFINE_LINK_FACTORY(StiOfLink, STI_OF_LINK)
DEFINE_LINK_FACTORY(LtiOfLink, LTI_OF_LINK)

/* ===================== END OF FILE ===================== */
