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

#include <unordered_map>

#include <opencog/atoms/bind/ScopeLink.h>

namespace opencog
{
class PatternMatchEngine;
class PatternMatchCallback;

/** \addtogroup grp_atomspace
 *  @{
 */

/// The ConcreteLink is used to specify a list of variables, and a
/// pattern (containing those variables) that is to be grounded
/// (satisfied).  Thus, it resembles a ScopeLink, with the difference
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
/// Given the initial list of variables and clauses, the constructors
/// extract the optional clauses and the dynamically-evaluatable clauses.
/// This also computes the connectivity diagram of the clauses.
///
/// It is assumed that the set of clauses form a single, connected
/// component; i.e. that the clauses are pair-wise connected by common,
/// shared variables, and that this pair-wise connection extends over
/// the entire set of clauses. There is no other restriction on the
/// connection topology; they can form any graph whatsoever (as long as
/// it is connected).
///
/// The (cog-satisfy) scheme call can ground this link, and return
/// a truth value.
class ConcreteLink : public ScopeLink
{
	// Private, locally scoped typedefs, not used outside of this class.
	// XX TODO Replace by unordered multimap...
	typedef std::vector<Handle> RootList;
	typedef std::map<Handle, RootList> ConnectMap;
	typedef std::pair<Handle, RootList> ConnectPair;

protected:
	/// The actual clauses. Set by validate_clauses()
	HandleSeq        _clauses;

	// The cnf_clauses are the clauses, but with the AbsentLink, NotLink
	// removed. This simplifies graph discovery, so that when they are
	// found, they can be rejected (e.g. are not absent)
	HandleSeq        _cnf_clauses;  // AbsentLink, NotLink removed!

	// The mandatory clauses must be grounded.
	HandleSeq        _mandatory;

	// The optional clauses don't have to be. This is where the
	// negated/absent clauses are held, so e.g. if these get grounded,
	// they might be rejected.
	std::set<Handle> _optionals;    // Optional clauses

	std::set<Handle> _evaluatable_terms;   // smallest term that is evaluatable
	std::set<Handle> _evaluatable_holders; // holds something evaluatable.
	std::set<Handle> _executable_terms;    // smallest term that is executable
	std::set<Handle> _executable_holders;  // holds something executable.

	// Maps; the value is the largest (evaluatable or executable)
	// term containing the variable. Its a multimap, because
	// a variable may appear in several different evaluatables.
	std::unordered_multimap<Handle,Handle> _in_evaluatable;
	std::unordered_multimap<Handle,Handle> _in_executable;

	// Any given atom may appear in one or more clauses. Given an atom,
	// the connectivy map tells you what clauses it appears in. It
	// captures how the clauses are connected to one-another, so that,
	// after one clause is solved, we know what parts of the unsolved
	// clauses already have a solution.
	ConnectMap       _connectivity_map;     // setup by make_connectivity_map()

	void unbundle_clauses(const Handle& body);
	void validate_clauses(std::set<Handle>& vars,
	                      HandleSeq& clauses);

	void extract_optionals(const std::set<Handle> &vars,
	                       const std::vector<Handle> &component);

	void unbundle_virtual(const std::set<Handle>& vars,
	                      const HandleSeq& clauses,
	                      HandleSeq& concrete_clauses,
	                      HandleSeq& virtual_clauses);

	void check_connectivity(const std::vector<HandleSeq>&);

	void make_connectivity_map(const HandleSeq&);
	void make_map_recursive(const Handle&, const Handle&);

	// Only derived classes can call this
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

	ConcreteLink(const std::set<Handle>& vars,
	             const VariableTypeMap& typemap,
	             const HandleSeq& component,
	             const std::set<Handle>& optionals);

	bool satisfy(PatternMatchCallback*) const;
	bool satisfy(PatternMatchCallback*, PatternMatchEngine*) const;

	void debug_print(const char* tag = "primary entry") const;
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
