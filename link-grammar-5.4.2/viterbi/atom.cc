/*************************************************************************/
/* Copyright (c) 2012 Linas Vepstas <linasvepstas@gmail.com>             */
/* All rights reserved                                                   */
/*                                                                       */
/* Use of the Viterbi parsing system is subject to the terms of the      */
/* license set forth in the LICENSE file included with this software.    */
/* This license allows free redistribution and use in source and binary  */
/* forms, with or without modification, subject to certain conditions.   */
/*                                                                       */
/*************************************************************************/

#include <algorithm>
#include <iostream>
#include <string>
#include <math.h>

#include "atom.h"
#include "utilities.h"
#include "compile-base.h"

namespace atombase {

using namespace std;

bool TV::operator==(const TV& other) const
{
	// The ULP for single-precision floating point is approx 1.0e-7.2
	if (fabs(other._strength - _strength) < 1.0e-6) return true;
	return false;
}

// ====================================================

// Single, global mutex for locking the incoming set.
std::mutex Atom::IncomingSet::_mtx;

// Destructor.
Atom::~Atom()
{
	drop_incoming_set();
}

/// Start tracking the incoming set for this atom.
/// An atom can't know what it's incoming set is, until this method
/// is called.  If this atom is added to any links before this call
/// is made, those links won't show up in the incoming set.
///
/// We don't automatically track incoming sets for two reasons:
/// 1) std::set takes up 48 bytes
/// 2) adding and remoiving uses up cpu cycles.
/// Thus, if the incoming set isn't needed, then don't bother
/// tracking it.
void Atom::keep_incoming_set()
{
	if (_incoming_set) return;
	_incoming_set = new IncomingSet;
}

/// Stop tracking the incoming set for this atom.
/// After this call, the incoming set for this atom can no longer
/// be queried; it si erased.
void Atom::drop_incoming_set()
{
	if (NULL == _incoming_set) return;
	std::lock_guard<std::mutex> lck (_incoming_set->_mtx);

	// Perform an explicit free and delete; this alleviates
	// pressure on the garbage collector.
	_incoming_set->_iset.clear();
	delete _incoming_set;
	_incoming_set = NULL;
}

// #define WEAK_POINTER_HASH 0x5555555555555555UL
#define WEAK_POINTER_HASH 0x0UL

/// Add an atom to the incoming set.
void Atom::insert_atom(Link* a)
{
	if (NULL == _incoming_set) return;
	std::lock_guard<std::mutex> lck (_incoming_set->_mtx);
	// Create a weak pointer, hidden from GC, by XOR'ing it.  I think
	// thi si the right thing to do.  The Boehm GC docs suggest that
	// GC_register_disappearing_link() be used, but I think that's only
	// when registering finalizers, which we don't do.
	WeakLinkPtr wa = (WeakLinkPtr) a;
	_incoming_set->_iset.insert(wa ^ WEAK_POINTER_HASH);
}

/// Remove an atom from the incoming set.
void Atom::remove_atom(Link* a)
{
	if (NULL == _incoming_set) return;
	std::lock_guard<std::mutex> lck (_incoming_set->_mtx);
	WeakLinkPtr wa = (WeakLinkPtr) a;
	_incoming_set->_iset.erase(wa ^ WEAK_POINTER_HASH);
}

/// Return a copy of the entire incoming set of this atom.
///
/// This returns a copy of the incoming set at the time it was called.
/// This call is thread-safe, and thread-consistent (i.e. the incoming
/// set is guaranteed not to get smaller for as long as the link is
/// held; it may, however, get larger, if there are any atoms creted
/// after this method returns.
Set* Atom::get_incoming_set() const
{
	if (NULL == _incoming_set) return new Set();
	std::function<Atom* (Link*)> filter = [&](Link* a) -> Atom*
	{
		return a;
	};
	return filter_iset(filter);
#ifdef EQUIVALENT_TO_ABOVE
	// The above expands out into this.
	std::unique_lock<std::mutex> lck (_incoming_set->_mtx);
	OutList oset;
	std::set<Link*>::iterator it = _incoming_set->_iset.begin();
	std::set<Link*>::iterator end = _incoming_set->_iset.end();
	for (; it != end; ++it)
	{
		oset.push_back(*it);
	}
	lck.unlock();
	return new Set(oset);
#endif
}

/// Like above, but filtering for type.
Set* Atom::get_incoming_set(AtomType type) const
{
	if (NULL == _incoming_set) return new Set();
	std::function<Atom* (Link*)> filter = [type](Link* a) -> Atom*
	{
		if (a->get_type() != type) return NULL;
		return a;
	};
	return filter_iset(filter);
}

// ====================================================

/// Add a nemed relation to the atom.
/// This method only creates named binary relations.
Relation* Atom::add_relation(const char* name, Atom* val)
{
	// We need to keep the incoming set, else the relation will not be findable.
	keep_incoming_set();
	return new Relation(name, this, val);
}

Set* Atom::filter_iset(std::function<Atom* (Link*)> filter) const
{
	if (NULL == _incoming_set) return new Set();
	std::unique_lock<std::mutex> lck (_incoming_set->_mtx);
	OutList oset;
	std::set<WeakLinkPtr>::iterator it = _incoming_set->_iset.begin();
	std::set<WeakLinkPtr>::iterator end = _incoming_set->_iset.end();
	for (; it != end; ++it)
	{
		WeakLinkPtr wa = *it;
		wa = wa ^ WEAK_POINTER_HASH;
		Link* l = (Link*) wa;
		Atom* a = filter(l);
		if (a) oset.push_back(a);
	}

	// Unlock the mutex before calling new, below.
	lck.unlock();
	return new Set(oset);
}

/// Get the set of all named relations
Set* Atom::get_relations(const char* name) const
{
	Label* lab = new Label(name);
	std::function<Atom* (Link*)> filter = [this, lab](Link* a) -> Atom*
	{
		if (RELATION != a->get_type()) return NULL;
		assert(1 < a->get_arity(), "Relation cannot be zero-ary");
		if (lab != a->get_outgoing_atom(0)) return NULL;
		assert(this == a->get_outgoing_atom(1), "Corrupted outgoing set for relation");
		return a;
	};
	return filter_iset(filter);
}

/// Get the set of all the values of the named relations
Set* Atom::get_relation_vals(const char* name) const
{
	Label* lab = new Label(name);
	std::function<Atom* (Link*)> filter = [this, lab](Link* a) -> Atom*
	{
		if (RELATION != a->get_type()) return NULL;
		assert(3 == a->get_arity(), "Expecting binary relation");
		if (lab != a->get_outgoing_atom(0)) return NULL;
		assert(this == a->get_outgoing_atom(1), "Corrupted outgoing set for relation");
		return a->get_outgoing_atom(2);
	};
	return filter_iset(filter);
}

// ====================================================

/// Constructor.  Place self into incoming set.
/// For every atom in the outgoing set of this link, add this link
/// to that atom's incoming set.
void Link::add_to_incoming_set()
{
	size_t arity = get_arity();
	for (size_t i=0; i<arity; i++)
		_oset[i]->insert_atom(this);
}

/// Place self into incoming sets, but only they are of type t.
/// For every atom of type t in the outgoing set of this link, add this
/// link to that atom's incoming set.
void Link::add_to_incoming_set(AtomType t)
{
	size_t arity = get_arity();
	for (size_t i=0; i<arity; i++)
		if (_oset[i]->get_type() == t)
			_oset[i]->insert_atom(this);
}

/// Remove self from the incoming sets, if they are of type t.
/// For every atom of type t in the outgoing set of this link, remove
/// this link from that atom's incoming set.
void Link::remove_from_incoming_set(AtomType t)
{
	size_t arity = get_arity();
	for (size_t i=0; i<arity; i++)
		if (_oset[i]->get_type() == t)
			_oset[i]->remove_atom(this);
}

/// Enable the tracking of incoming sets for atoms of type t.
/// For every atom of type t in the outgoing set of this link, enable
/// incoming-set tracking, and add this link to that atom's incoming set.
void Link::enable_keep_incoming_set(AtomType t)
{
	size_t arity = get_arity();
	for (size_t i=0; i<arity; i++)
		if (_oset[i]->get_type() == t)
		{
			_oset[i]->keep_incoming_set();
			_oset[i]->insert_atom(this);
		}
}

/// Disable the tracking of incoming sets for atoms of type t.
/// For every atom of type t in the outgoing set of this link, disable
/// incoming-set tracking.
void Link::disable_keep_incoming_set(AtomType t)
{
	size_t arity = get_arity();
	for (size_t i=0; i<arity; i++)
		if (_oset[i]->get_type() == t)
		{
			_oset[i]->drop_incoming_set();
		}
}

// Destructor.  Remove self from incoming set.
// Note: with garbage collection, this destructor is never called
// (and that is how things should be).  We keep it around here, for the
// rainy day when we swith to reference-counted pointers.
//
// Note also: if this ever was called during gc, e.g. as a finalizer,
// it will lead to deadlocks, since gc could get triggered by the call
// to insert_atom(), which aleady holds the same global lock that
// remove_atom() would use. i.e. using this in a finalizer will dead-lock
// (unless we convert to per-atom locks, which could be wasteful).
Link::~Link()
{
	// Alleviate pressure on the GC, whenever possible.
	size_t arity = get_arity();

	// Cast-away const so that we can trash the contents.
	OutList& os = (OutList&) _oset;
	for (size_t i=0; i<arity; i++)
	{
		os[i]->remove_atom(this);
		os[i] = NULL;
	}
	os.resize(0);
}

// ====================================================

bool Atom::operator==(const Atom* other) const
{
	if (!other) return false;
	if (other == this) return true;
	if (other->_type == this->_type and
	    other->_tv == this->_tv) return true;
	return false;
}

bool Node::operator==(const Atom* other) const
{
	if (!other) return false;
	if (other == this) return true;
	if (other->get_type() != this->get_type()) return false;
	if (not other->_tv.operator==(this->_tv)) return false;
	const Node* nother = dynamic_cast<const Node*>(other);
	if (nother->_name != this->_name)  return false;
	return true;
}

bool Link::operator==(const Atom* other) const
{
	if (!other) return false;
	if (other == this) return true;
	if (other->get_type() != this->get_type()) return false;
	if (not other->_tv.operator==(this->_tv)) return false;
	const Link* lother = dynamic_cast<const Link*>(other);
	if (lother->get_arity() != this->get_arity())  return false;
	for (size_t i=0; i<get_arity(); i++)
	{
		if (not (lother->_oset[i]->operator==(this->_oset[i]))) return false;
	}
	return true;
}


std::ostream& do_prt(std::ostream& out, const Atom* a, int ilvl)
{
	static const char *indent_str = "  ";
	const Node* n = dynamic_cast<const Node*>(a);
	if (n)
	{
		for (int i=0; i<ilvl; i++) cout << indent_str;
		out << n->get_type() << " : " << n->get_name();
		if (0.0f != n->_tv._strength)
			out << "    (" << n->_tv._strength << ")";
		out << endl;
		return out;
	}
	const Link* l = dynamic_cast<const Link*>(a);
	if (l)
	{
		for (int i=0; i<ilvl; i++) cout << indent_str;
		out << l->get_type() <<" :";
		if (0.0f != l->_tv._strength)
			out << "     (" << l->_tv._strength << ")";
		out << endl;

		ilvl++;
		size_t lsz = l->get_arity();
		for (size_t i=0; i < lsz; i++)
		{
			do_prt(out, l->get_outgoing_atom(i), ilvl);
		}
		return out;
	}

	out << "xxx-null-ptr-xxx";
	return out;
}

std::ostream& operator<<(std::ostream& out, const Atom* a)
{
	return do_prt(out, a, 0);
}

} // namespace atombase

