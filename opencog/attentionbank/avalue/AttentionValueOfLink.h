/*
 * opencog/attentionbank/avalue/AttentionValueOfLink.h
 *
 * Copyright (C) 2018 Linas Vepstas
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

#ifndef _OPENCOG_ATTENTION_VALUE_OF_LINK_H
#define _OPENCOG_ATTENTION_VALUE_OF_LINK_H

#include <opencog/atoms/core/ValueOfLink.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/// The AttentionValueOfLink returns the attention value on the
/// indicated atom.
///
class AttentionValueOfLink : public ValueOfLink
{
public:
	AttentionValueOfLink(const HandleSeq&, Type=ATTENTION_VALUE_OF_LINK);
	AttentionValueOfLink(const Link &l);

	// Return a pointer to the extracted value.
	virtual ValuePtr execute(AtomSpace*, bool);

	static Handle factory(const Handle&);
};

typedef std::shared_ptr<AttentionValueOfLink> AttentionValueOfLinkPtr;
static inline AttentionValueOfLinkPtr AttentionValueOfLinkCast(const Handle& h)
	{ return std::dynamic_pointer_cast<AttentionValueOfLink>(h); }
static inline AttentionValueOfLinkPtr AttentionValueOfLinkCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<AttentionValueOfLink>(a); }

#define createAttentionValueOfLink std::make_shared<AttentionValueOfLink>

// ====================================================================

/// The StiOfLink returns the STI of an attention value on the
/// indicated atom. (STI is the first of the sequence of floats).
///
class StiOfLink : public ValueOfLink
{
public:
	StiOfLink(const HandleSeq&, Type=STRENGTH_OF_LINK);
	StiOfLink(const Link &l);

	// Return a pointer to the extracted value.
	virtual ValuePtr execute(AtomSpace*, bool);

	static Handle factory(const Handle&);
};

typedef std::shared_ptr<StiOfLink> StiOfLinkPtr;
static inline StiOfLinkPtr StiOfLinkCast(const Handle& h)
	{ return std::dynamic_pointer_cast<StiOfLink>(h); }
static inline StiOfLinkPtr StiOfLinkCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<StiOfLink>(a); }

#define createStiOfLink std::make_shared<StiOfLink>

// ====================================================================

/// The LtiOfLink returns the LTI of an attention value on the
/// indicated atom. (LTI is the first of the sequence of floats).
///
class LtiOfLink : public ValueOfLink
{
public:
	LtiOfLink(const HandleSeq&, Type=CONFIDENCE_OF_LINK);
	LtiOfLink(const Link &l);

	// Return a pointer to the extracted value.
	virtual ValuePtr execute(AtomSpace*, bool);

	static Handle factory(const Handle&);
};

typedef std::shared_ptr<LtiOfLink> LtiOfLinkPtr;
static inline LtiOfLinkPtr LtiOfLinkCast(const Handle& h)
	{ return std::dynamic_pointer_cast<LtiOfLink>(h); }
static inline LtiOfLinkPtr LtiOfLinkCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<LtiOfLink>(a); }

#define createLtiOfLink std::make_shared<LtiOfLink>

/** @}*/
}

#endif // _OPENCOG_ATTENTION_VALUE_OF_LINK_H
