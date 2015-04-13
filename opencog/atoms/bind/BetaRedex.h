/*
 * opencog/atoms/BetaRedex.h
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

#ifndef _OPENCOG_BETA_REDEX_H
#define _OPENCOG_BETA_REDEX_H

#include <map>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atoms/bind/ScopeLink.h>
#include <opencog/query/PatternMatchCallback.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/// The BetaRedex link is used to specify a set of values that are to
/// be attached to the variables of a function named by a DefineLink.
/// Thus, a BetaRedex vaguely resembles a "function call", except that
/// no actual call is made (nor is any grounding or evaluation made).
/// The BetaRedex merely serves to specify the values that will be
/// passed to the named function. The name `BetaRedex` comes from the
/// concept of beta reduction in lambda calculus. The actual
/// substitution (beta-reduction) does not take place until the
/// `substitute` method is called.  Note, however, that the substitute
/// method only substitutes; it does not perform any evaluation,
/// grounding or calling. Thus, it is just a beta reduction, and nothing
/// more.
///
/// The intended use of this link type is to specify (name) patterns
/// that will be expanded during the course of a  pattern match.  The 
///
/// The BetaRedex link must have the following form:
///
///      BetaRedex
///          SomeNamingAtom
///          SomeOrderedLink
///             Argument_1
///             Argument_2
///             ...
///             Argument_N
///
/// The "SomeNamingAtom" will typically be a Node, a ConceptNode or a
/// PredicteNode, but it doesn't have to be.  However, it MUST appear
/// as the name of a DefineLink; that is, there MUST be a corresponding
/// DefineLink of the form
///
///      DefineLink
///          SomeNamingAtom
///          ScopeLink   ;;; Or SatisfactionLink or BindLink....
///              Vardecls
///              BodyAtom
///
/// The "SomeNamingAtom" must be exactly the same in the Define and the
/// BetaRedex; this is how the BetaRedex is able to find the defined
/// body.  The DefineLink does not have to be created before the
/// BetaRedex is created; however, it must exist by the time the
/// composition is used.  The DefineLink has to name either a
/// ScopeLink, or one of the link types inheriting from it (e.g. a
/// BindLink or SatisfactionLink).
///
/// The "SomeOrderedLink" holds the set of arguments associated with the
/// name.  These arguments may be "values", or they may be VariableNodes,
/// or a mixture of the two.  A "value" is anything that is not a
/// VariableNode.  If the arguments are values, then they MUST obey
/// the type restrictions specified in the ScopeLink.  If they are
/// VariableNodes, then they inherit the type restrictions from the
/// ScopeLink.
///
class BetaRedex : public Link
{
protected:
	void init(const HandleSeq&);
public:
	BetaRedex(const HandleSeq&,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	BetaRedex(const Handle& name, const Handle& args,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	BetaRedex(Link &l);

	// Return the name of the redex (the name of the defined function)
	const std::string& get_name(void);

	// Return the arguments of this redex.
	const HandleSeq& get_args(void) const;

	// Return the arguments of the lambda
	const HandleSeq& get_local_args(void) const;
	const std::set<Handle>& get_local_argset(void) const;

	// Return the lambda
	ScopeLinkPtr get_definition(void) const;

	// Return the substitution of the redex args in the lambda.
	Handle beta_reduce(void) const;

	//junk
	void satisfy(PatternMatchCallback*, const HandleSeq&);
};

typedef std::shared_ptr<BetaRedex> BetaRedexPtr;
static inline BetaRedexPtr BetaRedexCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<BetaRedex>(a); }
static inline BetaRedexPtr BetaRedexCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<BetaRedex>(a); }

// XXX temporary hack ...
#define createBetaRedex std::make_shared<BetaRedex>

/** @}*/
}

#endif // _OPENCOG_BETA_REDEX_H
