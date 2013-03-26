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

// XXX TODO:
// This class uses a HandleIndex as the map, and HandleIndex uses the
// ordered HandleSeq (i.e. a vector). For our purposes, we don't need
// a vector; a std::set would have been enough.  In fact, using std::set
// would make both insertion and removale a little bit simpler and faster.
// Oh well. Later.
IncomingIndex::IncomingIndex(void)
{
}

void IncomingIndex::resize()
{
}

void IncomingIndex::insertAtom(const Atom* a)
{
	const Link *l = dynamic_cast<const Link *>(a);
	if (NULL == l) return;

	Handle hin = a->getHandle();
	const std::vector<Handle>& oset = l->getOutgoingSet();
	for (std::vector<Handle>::const_iterator it = oset.begin(); it != oset.end(); it++)
	{
		Handle h = *it;
		
		const HandleSeq& oldset = idx.get(h);

		// Check to see if h is already listed; it should not be...
		for (HandleSeq::const_iterator oit = oldset.begin(); oit != oldset.end(); oit++)
		{
			OC_ASSERT(*oit != hin, "Same atom seems to be getting inserted twice!");
		}

		// Add hin to the incoming set of h.
		HandleSeq inset = oldset;
		inset.push_back(hin);
		idx.remove(h, oldset);
		idx.insert(h, inset);
	}
}

void IncomingIndex::removeAtom(const Atom* a)
{
	const Link *l = dynamic_cast<const Link *>(a);
	if (NULL == l) return;

	Handle hin = a->getHandle();
	const std::vector<Handle>& oset = l->getOutgoingSet();
	for (std::vector<Handle>::const_iterator it = oset.begin(); it != oset.end(); it++)
	{
		Handle h = *it;
		
		const HandleSeq& oldset = idx.get(h);
		HandleSeq inset = oldset;

		// Check to see if h is already listed; it should not be...
		HandleSeq::iterator oit = inset.begin();
		for (; oit != inset.end(); oit++)
		{
			if (*oit == hin) break;
		}
		OC_ASSERT(oit != inset.end(), "Can't find atom in the incoming set!");

		// Add hin to the incoming set of h.
		inset.erase(oit);
		idx.remove(h, oldset);
		idx.insert(h, inset);
	}
}

const HandleSeq& IncomingIndex::getIncomingSet(Handle h) const
{
	return idx.get(h);
}

void IncomingIndex::remove(bool (*filter)(const HandleSeq&))
{
	idx.remove(filter);
}

// ================================================================
