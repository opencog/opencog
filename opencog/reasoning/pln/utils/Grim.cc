/*
 * Copyright (C) 2002-2007 Novamente LLC
 * Copyright (C) 2008 by OpenCog Foundation
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

#include "../PLN.h"
#include "fim.h"
#include <stdio.h>
#include <opencog/util/macros.h>

using namespace std;
using namespace fim;

grim::grim()
: zeropat_index(257),
next_free_pat(258),
UPDATE_ALL_OLD_PATTERNS_UPON_NEW_INSERTION(true)
{
		zeropat = get_zeropat();
		pat[zeropat_index] = zeropat;
}

void grim::_copy(PAT dest, PAT src)
{
	for (int i = 0; i < PLN_CONFIG_PATTERN_LENGTH; i++)
		dest[i] = src[i];
}

/// Find common ground between two patterns. The slots not shared are marked with zeros (=variables)

int grim::create_common_pat(PAT a, PAT b, PAT* ret)
{
	*ret = new unsigned int[PLN_CONFIG_PATTERN_LENGTH];
	int count = 0;

	bool non_trivial = false;
	unsigned int found_pat_id = 0;

	for (int i = 0; i < PLN_CONFIG_PATTERN_LENGTH; i++)
		if (a[i] == b[i])
		{
			(*ret)[i] = a[i];
			count++;

			if (found_pat_id && a[i] && found_pat_id != a[i])
				non_trivial = true;
			if (found_pat_id != a[i])
				found_pat_id = a[i];
		}
		else 
			(*ret)[i] = 0;

	return (non_trivial ? count : 0);
}

/// The measurement of match is the number of matching non-variable atoms. However,
/// if each pattern contains a _differerent_ non-variable in a specific slot, then the match is zero.

int grim::match(PAT data, PAT wild_pattern)
{
	int count = 0;

	for (int i = 0; i < PLN_CONFIG_PATTERN_LENGTH; i++)
		if (wild_pattern[i])
		{
			if (wild_pattern[i] != data[i])
				return 0;
			count++;
		}

	return count; 
}

PAT grim::get_zeropat()
{
	PAT zeropat = new unsigned int[PLN_CONFIG_PATTERN_LENGTH];
	for (int i = 0; i < PLN_CONFIG_PATTERN_LENGTH; i++)
		zeropat[i] = 0;
	return zeropat;
}

/** From src pattern, mine the next non-existing pattern that could be extracted from it,
	ie. a pattern which can be found also in src and in an existing pattern,
	and if UPDATE_ALL_OLD_PATTERNS_UPON_NEW_INSERTION is true, the pattern base is
	also updated after the new pattern is added to the base.
*/

bool grim::create_next_pat(PAT src, pat_id* ret)
{
	PAT next = NULL;
	PAT longest = pat[zeropat_index];

	int longest_len=0;

	for (std::map<pat_id, PAT >::iterator i = 
		pat.begin(); i != pat.end(); i++)
	{
		int n = create_common_pat(src, i->second, &next);
		if (!longest || longest_len < n)
		{
			if (longest && longest_len)
				delete longest;
			longest = next;
			longest_len=n;
		}
		else
			delete next;
	}

	if (longest_len>0)
	{
		unsigned int longest_id = next_free_pat++;
		pat[(*ret = longest_id)] = longest;

		if (UPDATE_ALL_OLD_PATTERNS_UPON_NEW_INSERTION)
			for (std::map<pat_id, PAT >::iterator i = 
				pat.begin(); i != pat.end(); i++)
				if ((i->first != longest_id) && match(i->second, longest))
				{
					subst(pat[i->first], longest_id);

					rawPat2slimPat[i->first] = longest_id;
				}
	}
	else
		*ret = 0;

	return (longest_len>0);
}

/// From existing patterns, find the longest one that can be found from the pattern src.

bool grim::find_next_pat(PAT src, pat_id* ret)
{
	pat_id longest = zeropat_index;

	int longest_len=0;

	for (std::map<pat_id, PAT >::iterator i = 
		pat.begin(); i != pat.end(); i++)
	{
		int n = match(src, i->second);
		if (longest_len < n)
		{
			longest = i->first;
			longest_len = n;
		}
	}

	if (longest_len>0)
		*ret = longest;
	else
		*ret = 0;

	return (longest_len>0);
}

/** Substitute one pattern into the other. Currently we simply indicate
	"the presence of pattern #x in the pattern #y" by substituting the integer #x for each
	non-zero slot in #y.
	Eg. if #x is 20, x is [0,3,2] and y is [1,3,2], then the function returns the array
	[1, 20, 20]. The compression of this form is then a trivial task.
*/

bool grim::subst(PAT dest, pat_id src_i)
{
	PAT src = pat[src_i];

	for (int i =0; i < PLN_CONFIG_PATTERN_LENGTH; i++)
		if (src[i])
			dest[i] = src_i; //src[i];

	return false;
}

/// Printing operations

void grim::dump1(pat_id id, PAT p)
{
	printf("%d = ", id);
	for (int j=0;j<PLN_CONFIG_PATTERN_LENGTH ;j++)
		printf("%d ", p[j]);
	printf("\n---\n");
}

void grim::pat_dump()
{
	for (std::map<pat_id, PAT >::iterator i = 
		pat.begin(); i != pat.end(); i++)
	{
		printf("%d = ", i->first);
		for (int j=0;j<PLN_CONFIG_PATTERN_LENGTH;j++)
			printf("%d ", i->second[j]);
		printf("\n---\n");
	}
}

int grim::add(const PAT s)
{
	add(next_free_pat, s);

	return next_free_pat++;
}

/// The externally callable function that adds the pattern to the base by doing all the
/// pattern matching tasks.

int grim::add(pat_id id, const PAT s)
{
	pat_id next_pat;
	PAT new_s = new unsigned int[PLN_CONFIG_PATTERN_LENGTH];
	_copy(new_s, s);

	/// First: Find express the new pattern in terms of the existing ones:

	while (find_next_pat(new_s, &next_pat))
		subst(new_s, next_pat);

	/// 2nd: Find new patterns from the compressed pattern

	while (create_next_pat(new_s, &next_pat))
		subst(new_s, next_pat);

	/// Add the result to the pattern base

	pat[id] = new_s;
	
	return id;
}
