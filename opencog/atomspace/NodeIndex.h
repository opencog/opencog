/*
 * opencog/atomspace/NodeIndex.h
 *
 * Copyright (C) 2008,2015 Linas Vepstas <linasvepstas@gmail.com>
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

#ifndef _OPENCOG_NODEINDEX_H
#define _OPENCOG_NODEINDEX_H

#include <set>
#include <vector>

#include <opencog/atomspace/NameIndex.h>
#include <opencog/atomspace/types.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

/**
 * Implements an (type, name) index array of RB-trees (C++ set)
 * That is, given only the type and name of an atom, this will
 * return the corresponding handle of that atom.
 */
class NodeIndex
{
	private:
		std::vector<NameIndex> idx;

	public:
		NodeIndex();

		void insertAtom(const AtomPtr& a)
		{
			NameIndex &ni(idx[a->getType()]);
			ni.insertAtom(a);
		}
		void removeAtom(const AtomPtr& a)
		{
			NameIndex &ni(idx.at(a->getType()));
			ni.removeAtom(a);
		}
		void remove(bool (*)(const AtomPtr&));
		void resize();
		size_t size() const;

		const AtomPtr& getAtom(Type type, const std::string& str) const
		{
			const NameIndex &ni(idx.at(type));
			return ni.get(str);
		}

		UnorderedHandleSet getHandleSet(Type type, const std::string&, bool subclass) const;

		template <typename OutputIterator> OutputIterator
		getHandleSet(OutputIterator result,
		             Type type, const std::string& name, bool subclass) const
		{
			if (not subclass)
			{
				AtomPtr atom(getAtom(type, name));
				if (atom) *result++ = Handle(atom);
			}
			else
			{
				Type max = idx.size();
				for (Type s = 0; s < max; s++) {
					if (classserver().isA(s, type)) {
						AtomPtr atom(getAtom(s, name));
						if (atom) *result++ = atom->getHandle();
					}
				}
			}
			return result;
		}
};

/** @}*/
} //namespace opencog

#endif // _OPENCOG_NODEINDEX_H
