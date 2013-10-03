/*
 * opencog/atomspace/TargetTypeIndex.cc
 *
 * Copyright (C) 2008 Linas Vepstas <linasvepstas@gmail.com>
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

#include <opencog/atomspace/TargetTypeIndex.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Link.h>
#include <opencog/atomspace/TLB.h>

using namespace opencog;

TargetTypeIndex::TargetTypeIndex(void)
{
    resize();
}

void TargetTypeIndex::insertAtom(AtomPtr a)
{
	LinkPtr l(LinkCast(a));
	if (!l) return;

	Arity arity = l->getArity();
	if (0 == arity) return;

	const HandleSeq& oset = l->getOutgoingSet();
	std::set<Type> done_already;
	for (Arity i = 0; i < arity; i++)
	{
		Type type = TLB::getAtom(oset[i])->getType();
		if (done_already.find(type) == done_already.end())
		{
			insert(type, a->getHandle());
			done_already.insert(type);
		}
	}
}

void TargetTypeIndex::removeAtom(AtomPtr a)
{
	LinkPtr l(LinkCast(a));
	if (!l) return;

	Arity arity = l->getArity();
	if (0 == arity) return;

	const HandleSeq& oset = l->getOutgoingSet();
	std::set<Type> done_already;
	for (Arity i = 0; i < arity; i++)
	{
		Type type = TLB::getAtom(oset[i])->getType();
		if (done_already.find(type) == done_already.end())
		{
			remove(type, a->getHandle());
			done_already.insert(type);
		}
	}
}

// ================================================================
