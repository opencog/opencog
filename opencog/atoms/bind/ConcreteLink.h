/*
 * opencog/atoms/ConcreteLink.h
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

#ifndef _OPENCOG_CONCRETE_LINK_H
#define _OPENCOG_CONCRETE_LINK_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atoms/bind/LambdaLink.h>
#include <opencog/query/PatternMatchCallback.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/// The ConcreteLink is used to specify a list of variables, and a
/// pattern (containing those variables) that is to be grounded
/// (satisfied).  Thus, it resembles a LambdaLink, with the difference
/// being that it has a very specific semantics: the pattern is to be
/// grounded!
///
/// The body of the ConcreteLink is assumed to collection of clauses
/// to be satsified. Thus, the body is typically an AndLink, OrLink
/// or a SequentialAnd, depending on how they are to be satsified.
/// This is very much like a SatisfactionLink, with one exception:
/// NONE OF THE CLAUSES MAY BE VIRTUAL!! This restriction is
/// "artificial", in that it simplfies how pattern matching is done.
/// Thus, this class is not really intended for "general use", but
/// is for internal use only.
///  
/// The (cog-satisfy) scheme call can ground this link, and return
/// a truth value.
class ConcreteLink : public LambdaLink
{
	// Private, locally scoped typedefs, not used outside of this class.
	typedef std::vector<Handle> RootList;
	typedef std::map<Handle, RootList> ConnectMap;
	typedef std::pair<Handle, RootList> ConnectPair;

protected:
	/// The actual clauses. Set by validate_clauses()
	Handle _hclauses;
	HandleSeq        _cnf_clauses;
	HandleSeq        _mandatory;
	std::set<Handle> _optionals;
	std::set<Handle> _evaluatable;
	ConnectMap       _connectivity_map; // setup by make_connectivity_map()

	void make_connectivity_map(const Handle&, const Handle&);

	void ConcreteLink::holds_virtual(const std::set<Handle>& vars,
	                                 const Handle& clause,
	                                 bool& is_evaluatable,
	                                 bool& is_virtual);

	ConcreteLink(Type, const HandleSeq&,
	         TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	void init(void);

public:
	ConcreteLink(const HandleSeq&,
	         TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	ConcreteLink(const Handle& varcdecls, const Handle& body,
	         TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	         AttentionValuePtr av = AttentionValue::DEFAULT_AV());

	ConcreteLink(Link &l);
};

typedef std::shared_ptr<ConcreteLink> ConcreteLinkPtr;
static inline ConcreteLinkPtr ConcreteLinkCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<ConcreteLink>(a); }
static inline ConcreteLinkPtr ConcreteLinkCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<ConcreteLink>(a); }

// XXX temporary hack ...
#define createConcreteLink std::make_shared<ConcreteLink>

/** @}*/
}

#endif // _OPENCOG_CONCRETE_LINK_H
