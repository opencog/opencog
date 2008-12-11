/*
 * opencog/atomspace/ImportanceIndex.h
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

#ifndef _OPENCOG_IMPORTANCEINDEX_H
#define _OPENCOG_IMPORTANCEINDEX_H

#include <opencog/atomspace/AttentionValue.h>
#include <opencog/atomspace/FixedIntegerIndex.h>

namespace opencog
{
class Atom;
class HandleEntry;

/**
 * Implements an index with additional routines needed for managing 
 * short-term importance.
 */
class ImportanceIndex:
	public FixedIntegerIndex
{
	private:
		static HandleEntry* extractOld(AttentionValue::sti_t,
		                               Handle, bool recursive = false);
	public:
		ImportanceIndex(void);
		void insertHandle(Handle);
		void removeHandle(Handle);

		void updateImportance(Atom*, int);
		HandleEntry* decayShortTermImportance(void);
		HandleEntry* getHandleSet(AttentionValue::sti_t,
		                          AttentionValue::sti_t) const;
		static unsigned int importanceBin(short);
};

} //namespace opencog

#endif // _OPENCOG_IMPORTANCEINDEX_H
