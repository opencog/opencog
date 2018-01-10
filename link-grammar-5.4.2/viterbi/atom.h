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

#ifndef _ATOMBASE_ATOM_H
#define _ATOMBASE_ATOM_H

#include <iostream>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gc/gc_allocator.h>
#include <gc/gc_cpp.h>

#include "atom-types.h"

namespace atombase {

// Classes generally resembling those of the OpenCog AtomSpace
// These are tailored for use for the tracking task.

/**
 * TV (truth value): strength or likelihood of a link.
 *
 * Actually, we store the log-likelihood here, in units of bits,
 * rather than the probability.  This makes the numbers more
 * comprehensible and easier to read and debug.  To obtain the
 * probability (likelihood), just raise 2 to minus this value.
 *
 * Measuring in bits allows us to conflate ideas of energy, entropy,
 * complexity, cost.  In particular, long linkages will get a complexity
 * cost, whereas certain disjuncts have an innate cost, obtained from
 * entropy principles. These can be added together; they'e on the same
 * scale.
 */
class TV
{
	public:
		TV(float likeli=0.0f) : _strength(likeli) {}
		float _strength;
		bool operator==(const TV&) const;

		/// Log-likelihoods (costs, energies, entropies) add.
		TV& operator+=(const TV& other)
		{
			_strength += other._strength;
			return *this;
		}

		const TV operator+(const TV& other) const
		{
			return TV(*this) += other;
		}
};

/* Base class for Nodes and Links */
/**
 * Atoms are not mutable, except for the TV value. That is, you cannot
 * change the type of the atom.  In particular, all methods are const.
 *
 * The mutable TV value can cause problems.  In particular, when
 * propagating costs upwards when putting mixed expressions into DNF,
 * this mutability can mess things up.  The work-around for this is to
 * have a clone() function.  I'm not sure I like this.  Its ugly, because
 * of course, once an atom is in the atom space, its unique, and not clonable.
 * Ick.  Perhaps TV should not be mutable??
 *
 * All atoms are automatically garbage-collected.
 */
class Link;
class Relation;
class Set;
class Atom : public gc
{
	public:
		Atom(AtomType type, const TV& tv = TV()) :
			_tv(tv), _type(type), _incoming_set(NULL) {}
		virtual ~Atom();
		AtomType get_type() const { return _type; }
		TV _tv;

		void keep_incoming_set();
		void drop_incoming_set();
		Set* get_incoming_set() const;
		Set* get_incoming_set(AtomType) const;

		Relation* add_relation(const char*, Atom*);
		Set* get_relations(const char*) const;
		Set* get_relation_vals(const char*) const;

		virtual bool operator==(const Atom*) const;
		virtual Atom* clone() const = 0;
		Atom* upcaster();
	protected:
		friend class Link;  // wtf ???
		void insert_atom(Link*);
		void remove_atom(Link*);

		const AtomType _type;

		typedef unsigned long int WeakLinkPtr;
		struct IncomingSet : public gc
		{
				// Just right now, we will use a single shared mutex for all
				// locking on the incoming set.  If this causes too much
				// contention, then we can fall back to a non-global lock,
				// at the cost of 40 additional bytes per atom.
				static std::mutex _mtx;
				// incoming set is not tracked by garbage collector,
				// to avoid cyclic references.
				// std::set<ptr> uses 48 bytes (per atom).
				std::set<WeakLinkPtr, std::less<WeakLinkPtr>, gc_allocator<WeakLinkPtr> > _iset;
		};
		IncomingSet* _incoming_set;

		Set* filter_iset(std::function<Atom* (Link*)>) const;
};

/// Given an atom of a given type, return the C++ class of that type.
template<typename T>
T upcast(Atom* a)
{
	T t = dynamic_cast<T>(a);
	if (t) return t;
	return dynamic_cast<T>(a->upcaster());
}

typedef std::basic_string<char, std::char_traits<char>, gc_allocator<char> > NameString;
/**
 * A Node may be
 * -- a word (the std::string holds the word)
 * -- a link (the std::string holds the link)
 * -- a disjunct (the std::string holds the disjunct)
 * -- etc.
 * Nodes are immuatble; the name can be set but not changed.
 * Note: all methods are const.
 */
class Node : public Atom
{
	public:
		Node(const char* n, const TV& tv = TV())
			: Atom(NODE, tv), _name(n) {}

		Node(const NameString& n, const TV& tv = TV())
			: Atom(NODE, tv), _name(n) {}

		Node(AtomType t, const NameString& n, const TV& tv = TV())
			: Atom(t, tv), _name(n) {}

		const NameString& get_name() const { return _name; }

		virtual bool operator==(const Atom*) const;
		virtual Node* clone() const { return new Node(*this); }
	protected:
		const NameString _name;
};


/// All outgoing lists will be handled as vectors.
// Must use the bdw-gc allocator to track these pointers.
// If this is not done, the GC will fail to see the pointers here.
#if __cplusplus > 199711L
// using requires C++11
template <typename T>
using AtomList = std::vector<T, gc_allocator<Atom*> >;
typedef AtomList<Atom*> OutList;
#else
typedef std::vector<Atom*, gc_allocator<Atom*> > OutList;
#endif

/**
 * Links hold a bunch of atoms
 * Links are immutable; the outgoing set cannot be changed.
 * Note: all methods are const.
 */
class Link : public Atom
{
	public:
		// The main ctor
		Link(AtomType t, const OutList& oset, const TV& tv = TV())
			: Atom(t, tv), _oset(oset)
		{ add_to_incoming_set(); }
		Link(AtomType t, const TV& tv = TV())
			: Atom(t, tv)
		{ add_to_incoming_set(); }
		Link(AtomType t, Atom* a, const TV& tv = TV())
			: Atom(t, tv), _oset(1, a)
		{ add_to_incoming_set(); }
		Link(AtomType t, Atom* a, Atom*b, const TV& tv = TV())
			: Atom(t, tv), _oset(({OutList o(1,a); o.push_back(b); o;}))
		{ add_to_incoming_set(); }
		Link(AtomType t, Atom* a, Atom* b, Atom* c, const TV& tv = TV())
			: Atom(t, tv), _oset(({OutList o(1,a); o.push_back(b);
			                      o.push_back(c); o;}))
		{ add_to_incoming_set(); }
		Link(AtomType t, Atom* a, Atom* b, Atom* c, Atom* d, const TV& tv = TV())
			: Atom(t, tv), _oset(({OutList o(1,a); o.push_back(b);
			                      o.push_back(c); o.push_back(d); o;}))
		{ add_to_incoming_set(); }
		Link(AtomType t, Atom* a, Atom* b, Atom* c, Atom* d, Atom* e, const TV& tv = TV())
			: Atom(t, tv), _oset(({OutList o(1,a); o.push_back(b);
			                      o.push_back(c); o.push_back(d);
			                      o.push_back(e); o;}))
		{ add_to_incoming_set(); }
		virtual ~Link();

		size_t get_arity() const { return _oset.size(); }
		Atom* get_outgoing_atom(size_t pos) const { return _oset.at(pos); }
		const OutList& get_outgoing_set() const { return _oset; }

		void enable_keep_incoming_set(AtomType);
		void disable_keep_incoming_set(AtomType);
		void add_to_incoming_set();
		void add_to_incoming_set(AtomType);
		void remove_from_incoming_set(AtomType);
		Link* append(Atom*) const;
		Link* replace(Atom*, Atom*) const;

		virtual bool operator==(const Atom*) const;
		virtual Link* clone() const { return new Link(*this); }
	protected:
		// Outgoing set is const, not modifiable.
		const OutList _oset;
};


// An unhygienic for-each loop, to simplify iterating over
// the outgoing set.  I don't see a more elegant way to do this,
// just right now...
// Anyway, this implements the semantics "foreach VAR of TYPENAME in LNK"
#define foreach_outgoing(TYPENAME,VAR,LNK) \
	const atombase::Link* _ll_##VAR; \
	size_t _ii_##VAR, _ee_##VAR; \
	atombase::Atom* _aa_##VAR; \
	TYPENAME VAR; \
	for (_ll_##VAR = (LNK), _ii_##VAR = 0, \
	     _ee_##VAR = _ll_##VAR->get_arity(); \
	     _aa_##VAR = (_ii_##VAR < _ee_##VAR) ? \
	        _ll_##VAR->get_outgoing_atom(_ii_##VAR) : 0x0, \
	     VAR = dynamic_cast<TYPENAME>(_aa_##VAR), \
	     _ii_##VAR < _ee_##VAR; \
	     _ii_##VAR++)


std::ostream& operator<<(std::ostream& out, const Atom*);

} // namespace atombase

#endif // _ATOMBASE_ATOM_H
