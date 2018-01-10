/*************************************************************************/
/* Copyright (c) 2012, 2013 Linas Vepstas <linasvepstas@gmail.com>       */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the Viterbi parsing system is subject to the terms of the      */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#ifndef _ATOMBASE_COMPILE_H
#define _ATOMBASE_COMPILE_H

#include <map>
#include <stdexcept>
#include "atom.h"

namespace atombase {

// Classes that convert run-time atom types into compile-time static
// types, so that the compiler can check these for correctness.
// These are here purely for C++ programming convenience; the true
// structure that matters is the dynamic run-time (hyper-)graphs.

/// Simple string label node.
class Label : public Node
{
	public:
		Label(const NameString& n, const TV& tv = TV())
		   : Node(LABEL, n, tv) {}
};

/// Index, ID
/// Identification Node, holds one or several numeric ID values.
/// Intended primarily for debugging.
class Index : public Node
{
	public:
		Index(int a, const TV& tv = TV())
			: Node(INDEX, ({ char buff[80]; snprintf(buff, 80, "%d", a); buff;}), tv)
		{}
		Index(int a, int b, const TV& tv = TV())
			: Node(INDEX, ({ char buff[80]; snprintf(buff, 80, "%d, %d", a, b); buff;}), tv)
		{}
		Index(unsigned int a, int b, int c, const TV& tv = TV())
			: Node(INDEX, ({ char buff[80]; snprintf(buff, 80, "%u, %d, %d", a, b, c); buff;}), tv)
		{}
		Index(double a, const TV& tv = TV())
			: Node(INDEX, ({ char buff[80]; snprintf(buff, 80, "%20.16f", a); buff;}), tv)
		{}
};

/// Holder of one floating-point number
class Number : public Node
{
	public:
		Number(double a, const TV& tv = TV())
			: Node(NUMBER, ({ char buff[80]; snprintf(buff, 80, "%20.16g", a); buff;}), tv),
			_value(a)
		{}
		double get_value() const { return _value; }
	protected:
		double _value;
};

/// Named relation, as defined in model theory.
///
/// This consists of a name (Label), followed by an ordered sequence
/// of atoms.  The atoms are all related by the relation.  There are
/// many different ways of thinking about this:
/// 1) as a predicate: the relation is true/satisfied if it exists.
///    (the truth of which is given by the _tv, of course)
/// 2) as a graph of a function (in the sense of Bourbaki)
/// 3) as a (named) function, the first n atoms being the arguments,
///    the last being the value of the function for those arguments.
/// 4) the atom that is (more or less) equivalent to the OpenCog ExecutionLink
class Relation : public Link
{
	public:
		// Binary relation; add others if needed.
		Relation(Label* lab, Atom* arg, Atom* val, const TV& tv = TV())
			: Link(RELATION,  ({OutList o(1,lab); o.push_back(arg); o.push_back(val); o;}), tv)
		{}
		Relation(const char* name, Atom* arg, Atom* val, const TV& tv = TV())
			: Link(RELATION,  ({OutList o(1, new Label(name)); o.push_back(arg); o.push_back(val); o;}), tv)
		{}
};

/// Unordered multiset
/// A Set inherits fom Link, and is an unordered set of zero or more
/// atoms.  Properly speaking, it is a multi-set; the same atom may
/// appear more than once in the set.
class Set : public Link
{
	public:
		Set(const TV& tv = TV())
			: Link(SET, tv)
		{}
		Set(const OutList& ol, const TV& tv = TV())
			: Link(SET, ol, tv)
		{}
		Set(Atom* singleton, const TV& tv = TV())
			: Link(SET, OutList(1, singleton), tv)
		{}
		Set(Atom* a, Atom* b, const TV& tv = TV())
			: Link(SET, ({OutList o(1,a); o.push_back(b); o;}), tv)
		{}
		Set(Atom* a, Atom* b, Atom* c, const TV& tv = TV())
			: Link(SET, ({OutList o(1,a); o.push_back(b); o.push_back(c); o;}), tv)
		{}
		Set(Atom* a, Atom* b, Atom* c, Atom* d, const TV& tv = TV())
			: Link(SET, ({OutList o(1,a); o.push_back(b); o.push_back(c); o.push_back(d); o;}), tv)
		{}
		Set(Atom* a, Atom* b, Atom* c, Atom* d, Atom* e, const TV& tv = TV())
			: Link(SET, ({OutList o(1,a); o.push_back(b); o.push_back(c); o.push_back(d); o.push_back(e); o;}), tv)
		{}

	protected:
		/// The sole purpose of this ctor is to allow inheritance.
		Set(AtomType t, const TV& tv = TV())
			: Link(t, tv)
		{}
		Set(AtomType t, const OutList& oset, const TV& tv = TV())
			: Link(t, oset, tv)
		{}
		// Only for classes that inherit from Set
		Set(AtomType t, Atom* singleton, const TV& tv = TV())
			: Link(t, OutList(1, singleton), tv)
		{}
		Set(AtomType t, Atom* a, Atom* b, const TV& tv = TV())
			: Link(t, ({OutList o(1,a); o.push_back(b); o;}), tv)
		{}
		Set(AtomType t, Atom* a, Atom* b, Atom* c, const TV& tv = TV())
			: Link(t, ({OutList o(1,a); o.push_back(b); o.push_back(c); o;}), tv)
		{}
		Set(AtomType t, Atom* a, Atom* b, Atom* c, Atom* d, const TV& tv = TV())
			: Link(t, ({OutList o(1,a); o.push_back(b); o.push_back(c); o.push_back(d); o;}), tv)
		{}
		Set(AtomType t, Atom* a, Atom* b, Atom* c, Atom* d, Atom* e, const TV& tv = TV())
			: Link(t, ({OutList o(1,a); o.push_back(b); o.push_back(c); o.push_back(d); o.push_back(e); o;}), tv)
		{}

	public:
		// See the C file for documentation
		Set* flatten() const { return new Set(flatset(), _tv); }
		Atom* super_flatten() const;

		// Set-union (append) other set  to this set.
		virtual Set* sum(const Set*) const;

      virtual Set* clone() const { return new Set(*this); }

		Set* append(Atom* a) const { return dynamic_cast<Set*>(Link::append(a)); }
		Set* replace(Atom* novi, Atom* ctari) const {
			return dynamic_cast<Set*>(Link::replace(novi, ctari));
		}

	protected:
		OutList flatset() const;
};


/// Unique set. An atom may appear at most once in the outgoing set.
/// Duplicates are removed during construction.
class Uniq : public Set
{
	public:
		Uniq(const TV& tv = TV())
			: Set(UNIQ, tv)
		{}
		Uniq(const OutList& ol, const TV& tv = TV())
			: Set(UNIQ, uniqify(ol), tv)
		{}
		Uniq(Atom* singleton, const TV& tv = TV())
			: Set(UNIQ, uniqify(OutList(1, singleton)), tv)
		{}
		Uniq(Atom* a, Atom* b, const TV& tv = TV())
			: Set(UNIQ, uniqify(({OutList o(1,a); o.push_back(b); o;})), tv)
		{}
		Uniq(Atom* a, Atom* b, Atom* c, const TV& tv = TV())
			: Set(UNIQ, uniqify(({OutList o(1,a); o.push_back(b); o.push_back(c); o;})), tv)
		{}

		// Special copy constructor
		Uniq(Set* sset)
			: Set(UNIQ, uniqify(sset->get_outgoing_set()), sset->_tv)
		{}

		// XXX TODO we really should overload append, and enforce uniqueness. ...
	protected:
		static OutList uniqify(const OutList& ol);
};

/// Ordered sequence
/// Seq inherits from Set, and is an ordered sequence of zero or more
/// atoms.
class Seq : public Set
{
	public:
		Seq()
			: Set(SEQ)
		{}
		Seq(const OutList& ol, const TV& tv = TV())
			: Set(SEQ, ol, tv)
		{}
		Seq(Atom* singleton, const TV& tv = TV())
			: Set(SEQ, OutList(1, singleton), tv)
		{}
		Seq(Atom* a, Atom* b, const TV& tv = TV())
			: Set(SEQ, ({OutList o(1,a); o.push_back(b); o;}), tv)
		{}

		// See the Set class for documentation
		Seq* flatten() const { return new Seq(flatset(), _tv); }

      virtual Seq* clone() const { return new Seq(*this); }

		Seq* append(Atom* a) const { return dynamic_cast<Seq*>(Link::append(a)); }

		/// Get the first and last atoms in the sequence
		Atom* get_first() const {
			if (0 < get_arity()) return get_outgoing_atom(0);
			else return NULL;
		}
		Atom* get_last() const {
			size_t sz = get_arity();
			if (0 < sz) return get_outgoing_atom(sz-1);
			else return NULL;
		}


	protected:
		/// The sole purpose of this ctor is to allow inheritance.
		Seq(AtomType t)
			: Set(t)
		{}
		Seq(AtomType t, const OutList& oset, const TV& tv = TV())
			: Set(t, oset, tv)
		{}
		Seq(AtomType t, Atom* a, Atom* b, const TV& tv = TV())
			: Set(t, ({OutList o(1,a); o.push_back(b); o;}), tv)
		{}
		Seq(AtomType t, Atom* a, Atom* b, Atom* c, const TV& tv = TV())
			: Set(t, ({OutList o(1,a); o.push_back(b); o.push_back(c); o;}), tv)
		{}
		Seq(AtomType t, Atom* a, Atom* b, Atom* c, Atom* d, const TV& tv = TV())
			: Set(t, ({OutList o(1,a); o.push_back(b); o.push_back(c); o.push_back(d); o;}), tv)
		{}
};

/// Unordered OR of all children
class Or : public Set
{
	public:
		Or(const TV& tv = TV())
			: Set(OR, tv)
		{}
		Or(const OutList& ol, const TV& tv = TV())
			: Set(OR, ol, tv)
		{}
		Or(Atom* singleton, const TV& tv = TV())
			: Set(OR, OutList(1, singleton), tv)
		{}
		Or(Atom* a, Atom* b, const TV& tv = TV())
			: Set(OR, ({OutList o(1,a); o.push_back(b); o;}), tv)
		{}
		Or(Atom* a, Atom* b, Atom* c, const TV& tv = TV())
			: Set(OR, ({OutList o(1,a); o.push_back(b); o.push_back(c); o;}), tv)
		{}

		// Return disjunctive normal form (DNF)
		Atom* disjoin() const;

		// See the Set class for documentation
		Or* flatten() const { return new Or(flatset(), _tv); }

		// Remove repeated entries
		Or* uniq() const;

      virtual Or* clone() const { return new Or(*this); }

		Or* append(Atom* a) const { return dynamic_cast<Or*>(Link::append(a)); }
};

/// Ordered sequence
/// And inherits from Seq, since the order of the atoms in
/// its outgoing set is important.
class And : public Seq
{
	public:
		And()
			: Seq(AND)
		{}
		And(const OutList& ol, const TV& tv = TV())
			: Seq(AND, ol, tv)
		{}
		And(Atom* singleton, const TV& tv = TV())
			: Seq(AND, OutList(1, singleton), tv)
		{}
		And(Atom* a, Atom* b, const TV& tv = TV())
			: Seq(AND, ({OutList o(1,a); o.push_back(b); o;}), tv)
		{}
		And(Atom* a, Atom* b, Atom* c, const TV& tv = TV())
			: Seq(AND, ({OutList o(1,a); o.push_back(b); o.push_back(c); o;}), tv)
		{}


		// Return disjunctive normal form (DNF)
		// Does not modify this atom; just returns a new one.
		Atom* disjoin();

		// See the Set class for documentation
		And* flatten() const { return new And(flatset(), _tv); }

		/// Remove optional clauses.
		/// XXX Perhaps this should not be a method on this class...
		Atom* clean() const;

      virtual And* clone() const { return new And(*this); }

		And * append(Atom* a) const { return dynamic_cast<And*>(Link::append(a)); }
};

} // namespace atombase


#endif // _ATOMBASE_COMPILE_H
