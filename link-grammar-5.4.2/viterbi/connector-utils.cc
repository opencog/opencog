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

#include <ctype.h>

#include <algorithm>
#include <iostream>

#include "utilities.h"

#include "compile.h"
#include "connector-utils.h"

namespace link_grammar {
namespace viterbi {

/**
 * Compare two connector strings, see if they mate.
 * Return true if they do, else return false.
 * All upper-case letters must match exactly.
 * Lower case letters must match exactly, or must match wildcard '*'.
 * All strings are implicitly padded with an infinite number of
 * wild-cards on the right; thus, only need to compare against the
 * shorter of the * two strings.
 */
bool conn_match(const NameString& ls, const NameString& rs)
{
	char ldir = *ls.rbegin();
	char rdir = *rs.rbegin();
	assert (ldir == '+' or ldir == '-', "Bad word direction (l)");
	assert (rdir == '+' or rdir == '-', "Bad word direction (r)");

	// Direction signs must couple.
	if ('+' == ldir and '-' != rdir) return false;
	if ('-' == ldir and '+' != rdir) return false;

	// Captial letters must match. Wildcards match anything lower-case.
	NameString::const_iterator lp = ls.begin();
	NameString::const_iterator rp = rs.begin();
	size_t lslen = ls.size();
	size_t rslen = rs.size();
	size_t minlen = std::min(lslen, rslen);
	size_t len = minlen - 1;  // -1 for direction
	while (0 < len)
	{
		if (*lp != *rp)
		{
			// All upper-case letters must match exactly!
			if (isupper(*lp) or isupper(*rp)) return false;
			// Wild-card matches anything.
			if ('*' != *lp and '*' != *rp) return false;
		}
		lp++;
		rp++;
		len--;
	}
	// If the longer string is sill upper-case .. ouch
	if ((minlen < lslen) and isupper(*lp)) return false;
	if ((minlen < rslen) and isupper(*rp)) return false;

	return true;
}

/**
 * Merge two connector strings to create the linkage string.
 * It is assumed that the two connectors mate; no error checking is
 * done to detect if they don't. 
 * Example:   W*n+ merged with Wi*dy- gives Windy
 */
NameString conn_merge(const NameString& ls, const NameString& rs)
{
	NameString::const_iterator lp = ls.begin();
	NameString::const_iterator rp = rs.begin();
	size_t len = -1 + std::max(ls.size(), rs.size());
	NameString merger;
	merger.reserve(len);
	while (0 < len)
	{
		if (lp == ls.end())
			merger.push_back(*rp);
		else if (rp == rs.end())
			merger.push_back(*lp);
		else if ('*' != *lp and '+' != *lp and '-' != *lp)
			merger.push_back(*lp);
		else if ('+' != *rp and '-' != *rp)
			merger.push_back(*rp);
		else
			merger.push_back('*');
			
		if (lp != ls.end()) lp++;
		if (rp != rs.end()) rp++;
		len--;
	}
	return merger;
}

// =============================================================

/// Return true if the indicated atom is an optional clause.
/// else return false.
bool is_optional(Atom *a)
{
	AtomType ty = a->get_type();
	if (CONNECTOR == ty)
	{
		Connector* n = dynamic_cast<Connector*>(a);
		if (n->is_optional())
			return true;
		return false;
	}
	assert (OR == ty or AND == ty, "Must be boolean junction");

	foreach_outgoing(Atom*, atom, dynamic_cast<Link*>(a))
	{
		bool c = is_optional(atom);
		if (OR == ty)
		{
			// If anything in OR is optional, the  whole clause is optional.
			if (c) return true;
		}
		else
		{
			// ty is AND
			// If anything in AND is isn't optional, then something is required
			if (!c) return false;
		}
	}
	
	// All disj were required.
	if (OR == ty) return false;

	// All conj were optional.
	return true;
}

// ===================================================================

// Utility for below. See description given there.
static Atom* trim_left_pointers(Atom* a)
{
	Connector* ct = dynamic_cast<Connector*>(a);
	if (ct)
	{
		if (ct->is_optional())
			return a;
		char dir = ct->get_direction();
		if ('-' == dir) return NULL;
		if ('+' == dir) return a;
		assert(0, "Bad word direction (t)");
	}

	AtomType ty = a->get_type();
	assert (OR == ty or AND == ty, "Must be boolean junction");

	// Note: With the new DNF style of processing esewhere in the code,
	// it will never be the case that the block of code will be hit.
	// None-the-less, the code below works great for arbitrarily-nested
	// connector sets (i.e. sets that are not in any normal form) and
	// so its left here in case you need it.
	if (OR == ty)
	{
		OutList new_ol;
		foreach_outgoing(Atom*, ota, dynamic_cast<Link*>(a))
		{
			Atom* new_ota = trim_left_pointers(ota);
			if (new_ota)
				new_ol.push_back(new_ota);
		}
		if (0 == new_ol.size())
			return NULL;

		// The result of trimming may be multiple empty nodes. 
		// Remove all but one of them.
		bool got_opt = false;
		size_t nsz = new_ol.size();
		for (size_t i = 0; i < nsz; i++)
		{
			Connector* c = dynamic_cast<Connector*>(new_ol[i]);
			if (c and c->is_optional())
			{
				if (!got_opt)
					got_opt = true;
				else
					new_ol.erase(new_ol.begin() + i);
			}
		}

		if (1 == new_ol.size())
		{
			// If the entire OR-list was pruned down to one connector,
			// and that connector is the empty connector, then it
			// "connects to nothing" on the left, and should be removed.
			Connector* c = dynamic_cast<Connector*>(new_ol[0]);
			if (c and c->is_optional())
				return NULL;
			return new_ol[0];
		}

		return new Link(OR, new_ol);
	}

	// If we are here, then it an AND-list, and all connectors are
	// mandatory, unless they are optional.  So fail, if the
	// connectors that were trimmed were not optional.
	OutList new_ol;
	foreach_outgoing(Atom*, ota, dynamic_cast<Link*>(a))
	{
		Atom* new_ota = trim_left_pointers(ota);
		if (new_ota)
			new_ol.push_back(new_ota);
		else
			if (!is_optional(ota))
				return NULL;
	}

	if (0 == new_ol.size())
		return new And();

	if (1 == new_ol.size())
		return new_ol[0];

	return new And(new_ol);
}

/// Trim away all optional left pointers (connectors with - direction)
/// If there are any non-optional left-pointers, then return NULL.
///
/// If all of the connectors were optional left-pointers, then they
/// are all trimmed away, and a single, empty AND is returned.
WordCset* cset_trim_left_pointers(WordCset* wrd_cset)
{
	Atom* trimmed = trim_left_pointers(wrd_cset->get_cset());
	if (!trimmed)
		return NULL;
	return new WordCset(wrd_cset->get_word(), trimmed);
}

} // namespace viterbi
} // namespace link-grammar

