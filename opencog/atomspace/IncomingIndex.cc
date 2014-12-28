/*
 * opencog/atomspace/IncomingIndex.cc
 *
 * Copyright (C) 2008,2013 Linas Vepstas <linasvepstas@gmail.com>
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

#include <opencog/util/oc_assert.h>

#include <opencog/atomspace/IncomingIndex.h>
#include <opencog/atomspace/Link.h>

using namespace opencog;

IncomingIndex::IncomingIndex(void)
{
}

void IncomingIndex::resize()
{
}

void IncomingIndex::insertAtom(AtomPtr a)
{
	LinkPtr l(LinkCast(a));
	if (NULL == l) return;

	Handle hin = a->getHandle();
	const std::vector<Handle>& oset = l->getOutgoingSet();
	for (std::vector<Handle>::const_iterator it = oset.begin(); it != oset.end(); ++it)
	{
		Handle h = *it;
		
		const UnorderedHandleSet& oldset = idx.get(h);

		// Check to see if h is already listed; it should not be...
		// Unless a given handle appears twice in the outgoing set
		// of a link, in which case we should ignore the second
		// (and other) instances.
		UnorderedHandleSet::const_iterator oit = oldset.begin();
		for (; oit != oldset.end(); ++oit)
		{
			// OC_ASSERT(*oit != hin, "Same atom seems to be getting inserted twice!");
			if (*oit == hin) break;
		}
		if (oit != oldset.end()) continue;

		// Add hin to the incoming set of h.
		UnorderedHandleSet inset = oldset;
		inset.insert(hin);
		idx.remove(h, oldset);
		idx.insert(h, inset);
	}
}

void IncomingIndex::removeAtom(AtomPtr a)
{
	LinkPtr l(LinkCast(a));
	if (NULL == l) return;

	Handle hin = a->getHandle();
	const std::vector<Handle>& oset = l->getOutgoingSet();
	for (std::vector<Handle>::const_iterator it = oset.begin(); it != oset.end(); ++it)
	{
		Handle h = *it;
		
		const UnorderedHandleSet& oldset = idx.get(h);
		UnorderedHandleSet inset = oldset;

		UnorderedHandleSet::iterator oit = inset.begin();
		for (; oit != inset.end(); ++oit)
		{
			if (*oit == hin) break;
		}

		// Check to see if h was already deleted; it could happen that
		// h appears twice, or more, in oset.  The first time we see it,
		// it gets removed from the index.  The second and later times,
		// we do nothing.
		if (oit != inset.end())
		{
			// Remove hin from the incoming set of h.
			inset.erase(oit);
			idx.remove(h, oldset);
			idx.insert(h, inset);
		}
	}
}

const UnorderedHandleSet& IncomingIndex::getIncomingSet(Handle h) const
{
	return idx.get(h);
}

void IncomingIndex::remove(bool (*filter)(Handle))
{
	idx.remove(filter);
}

// ================================================================

IncomingIndex::iterator IncomingIndex::begin(Handle h) const
{
	iterator it(h);
	if (Handle::UNDEFINED != h)
		it._iset = idx.get(h);
	it._s = it._iset.begin();
	it._e = it._iset.end();
	return it;
}

IncomingIndex::iterator IncomingIndex::end(void) const
{
	iterator it(Handle::UNDEFINED);
	it._s = it._e = it._iset.end();
	return it;
}

// Note that his iterator makes a copy ofthe incoming set,
// and is thusstable against insertions and deletions from
// the incoming set in the parent class.
IncomingIndex::iterator::iterator(Handle h)
{
	_h = h;
}

IncomingIndex::iterator& IncomingIndex::iterator::operator=(iterator v)
{
	_h = v._h;
	_iset = v._iset;
	_s = v._s;
	_e = v._e;
	return *this;
}

Handle IncomingIndex::iterator::operator*(void)
{
	if (_s == _e) return Handle::UNDEFINED;
	return *_s;
}

bool IncomingIndex::iterator::operator==(iterator v)
{
	// If v is end(), then _h is undefined ...
	if (v._h != Handle::UNDEFINED and
	      _h != Handle::UNDEFINED and _h != v._h) return false;
	return _s == v._s;
}

bool IncomingIndex::iterator::operator!=(iterator v)
{
	if (v._h != Handle::UNDEFINED and
	      _h != Handle::UNDEFINED and _h != v._h) return true;
	return _s != v._s;
}

IncomingIndex::iterator& IncomingIndex::iterator::operator++()
{
	return operator++(1);
}

IncomingIndex::iterator& IncomingIndex::iterator::operator++(int i)
{
	while (0 < i and  _s != _e) { ++_s; --i; }
	return *this;
}

// ================================================================
