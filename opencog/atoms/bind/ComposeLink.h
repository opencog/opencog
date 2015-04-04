/*
 * opencog/atoms/ComposeLink.h
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

#ifndef _OPENCOG_COMPOSE_LINK_H
#define _OPENCOG_COMPOSE_LINK_H

#include <map>

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atoms/bind/LambdaLink.h>
#include <opencog/query/PatternMatchCallback.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 *
 * Experimental ComposeLink class. This is a rough sketch for how things
 * like this might be done. It is not necessarily a good idea, and might
 * be replaced by something completely different, someday ...
 */

/// The ComposeLink is used to specify a set of values that are to be
/// attached to the variables of a function named by a DefineLink.
/// Thus, a ComposeLink vaguely resembles a "function call", except that
/// no actual call is made (nor is any grounding or evaluation made).
/// The ComposeLink merely serves to specify the values that will be
/// passed to the named function. Perhaps "SubstitutionLink" might be a
/// better name: it specifies what will e substituted.  The actual
/// substitution does not take place until the `substitute` method is
/// called.  Note, however, that the substitute
/// substitutes values for variables.  It does not actually perform any
/// calling, evaluation or grounding; it merely serves to identify the
/// named function with which it is to be composed.
///
/// The ComposeLink must have the following form:
///
///      ComposeLink
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
///          LambdaLink   ;;; Or SatisfactionLink or BindLink....
///              Vardecls
///              BodyAtom
///
/// The "SomeNamingAtom" must be exactly the same in the Define and the
/// Compose; this is how the ComposeLink is able to find the defined
/// body.  The DefineLink does not have to be created before the
/// ComposeLink is created; however, it must exist by the time the
/// composition is used.  The DefineLink has to name either a
/// LambdaLink, or one of the link types inheriting from it (e.g. a
/// BindLink or SatisfactionLink).
///
/// The "SomeOrderedLink" holds the set of arguments associated with the
/// name.  These arguments may be "values", or they may be
/// VariableNodes, or a mixture of the two.  A "value" is anything that
/// is not a VariableNode.  If the arguments are values, then they MUST
/// obey the type restrictions specified in the LambdaLink.  If they are
/// VariableNodes, then they inherit the type restrictions from the
/// LambdaLink.
///
/// The intended use of this link type is to specify (name) patterns
/// that will be expanded during the course of a  pattern match.  The 
class ComposeLink : public Link
{
protected:
	void init(const HandleSeq&);
public:
	ComposeLink(const HandleSeq&,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	ComposeLink(const Handle& varcdecls, const Handle& body,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	ComposeLink(Link &l);

	// Return the arguments that this link was defined with.
	const HandleSeq& get_args(void) const;
	const HandleSeq& get_local_args(void) const;
	LambdaLinkPtr get_definition(void) const;
	Handle compose(void) const;
	void satisfy(PatternMatchCallback*, const HandleSeq&);
};

typedef std::shared_ptr<ComposeLink> ComposeLinkPtr;
static inline ComposeLinkPtr ComposeLinkCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<ComposeLink>(a); }
static inline ComposeLinkPtr ComposeLinkCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<ComposeLink>(a); }

// XXX temporary hack ...
#define createComposeLink std::make_shared<ComposeLink>

/** @}*/
}

#endif // _OPENCOG_COMPOSE_LINK_H
