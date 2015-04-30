/*
 * opencog/atoms/SatisfactionLink.h
 *
 * Copyright (C) 2015 Linas Vepstas
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

#ifndef _OPENCOG_SATISFACTION_LINK_H
#define _OPENCOG_SATISFACTION_LINK_H

#include <opencog/atoms/bind/ConcreteLink.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/// The SatisfactionLink is used to specify a list of variables, and a
/// pattern (containing those variables) that is to be grounded
/// (satisfied).  Thus, it resembles a ScopeLink, with the difference
/// being that it has a very specific semantics: the pattern is to be
/// grounded!
///
/// The body of the ConcreteLink is assumed to collection of clauses
/// to be satsified. Thus, the body is typically an AndLink, OrLink
/// or a SequentialAnd, depending on how they are to be satsified.
/// This is very much like a ConcreteLink, except that it may contain
/// clauses that are virtual (e.g. GreaterThanLink, or EvaluationLinks
/// with GroundedPredicateNodes).
///
/// It is similar to a BindLink, except that a BindLink also causes an
/// implication to be performed: after a grounding is found, the
/// BindLink then causes the implication to run with the resultant
/// grounding.  The SatisfactionLink does not specify what should happen
/// with the grounding, although the (cog-satisfy) scheme call returns
/// a truth value.
class SatisfactionLink : public ConcreteLink
{
protected:
	/// The graph components. Set by validate_clauses()
	/// "virtual" clauses are those that contain virtual links.
	/// "fixed" clauses are those that do not.
	/// The list of component_vars are the variables that appear
	/// in the corresponding component.
	HandleSeq _fixed;
	size_t _num_virts;
	HandleSeq _virtual;
	size_t _num_comps;
	HandleSeq _components;

	SatisfactionLink(Type, const HandleSeq&,
	         TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	void init(void);
	void setup_sat_body(void);

public:
	SatisfactionLink(const Handle& body,
	         TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	SatisfactionLink(const Handle& varcdecls, const Handle& body,
	         TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	SatisfactionLink(const HandleSeq&,
	         TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	SatisfactionLink(Link &l);

	SatisfactionLink(const std::set<Handle> &vars,
	                 const HandleSeq& clauses);

	// XXX temp hack till thigs get sorted out; remove this method later.
	const Pattern& get_pattern(void) { return _pat; }

	bool satisfy(PatternMatchCallback&) const;
};

typedef std::shared_ptr<SatisfactionLink> SatisfactionLinkPtr;
static inline SatisfactionLinkPtr SatisfactionLinkCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<SatisfactionLink>(a); }
static inline SatisfactionLinkPtr SatisfactionLinkCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<SatisfactionLink>(a); }

// XXX temporary hack ...
#define createSatisfactionLink std::make_shared<SatisfactionLink>

/** @}*/
}

#endif // _OPENCOG_SATISFACTION_LINK_H
