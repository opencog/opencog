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

#ifndef ARI_FIM_H
#define ARI_FIM_H

#include <vector>
#include <set>
#include <map>

namespace fim
{
/// Use integer patterns

typedef unsigned int* PAT;
#define PLN_CONFIG_PATTERN_LENGTH 13

/// C++ helper structures & functions:

template<int LEN>
struct lessi : public std::binary_function<int*, int*, bool>
{
	bool operator()(int* lhs, int* rhs)
	{
		for (int i = 0; i < LEN; i++)
			if (lhs[i] < rhs[i])
				return true;
			else if (lhs[i] > rhs[i])
				return false;

		return true;
	}
};

/** The patterns are identified via a simple integer ID into which we can std::map atom types
as well as node type/name pairs, if needed. The indexing of new patterns begins from
the initial value of next_free_pat, but this is obviously possible to do in a more
dynamic / flexible way.
*/

typedef unsigned int pat_id;

class grim
{
protected:
pat_id zeropat_index;
pat_id next_free_pat;

/// A very rough way to decide the policy for keeping the pattern base up-to-date:

const bool UPDATE_ALL_OLD_PATTERNS_UPON_NEW_INSERTION;

/// Find common ground between two patterns. The slots not shared are marked with zeros (=variables)

int create_common_pat(PAT a, PAT b, PAT* ret);
/// The measurement of match is the number of matching non-variable atoms. However,
/// if each pattern contains a _differerent_ non-variable in a specific slot, then the match is zero.

int match(PAT data, PAT wild_pattern);
PAT get_zeropat();

/** From src pattern, mine the next non-existing pattern that could be extracted from it,
	ie. a pattern which can be found also in src and in an existing pattern,
	and if UPDATE_ALL_OLD_PATTERNS_UPON_NEW_INSERTION is true, the pattern base is
	also updated after the new pattern is added to the base.
*/

bool create_next_pat(PAT src, pat_id* ret);

/// From existing patterns, find the longest one that can be found from the pattern src.

bool find_next_pat(PAT src, pat_id* ret);

/** Substitute one pattern into the other. Currently we simply indicate
	"the presence of pattern #x in the pattern #y" by substituting the integer #x for each
	non-zero slot in #y.
	Eg. if #x is 20, x is [0,3,2] and y is [1,3,2], then the function returns the array
	[1, 20, 20]. The compression of this form is then a trivial task.
*/

bool subst(PAT dest, pat_id src_i);

public:
	grim();

/// The "no pattern" pattern. Zeros in a pattern correspond to variables, so
/// if we want to use the current atom types, we must make sure that no atom type is
/// defined as number zero.

PAT zeropat;

std::map<pat_id, PAT> pat;

/// Something like the following can be used to make frequency calculations. Currently not used.

std::map<PAT, int, lessi<PLN_CONFIG_PATTERN_LENGTH> > pat2count;

std::map<pat_id, pat_id> rawPat2slimPat;

static void _copy(PAT dest, PAT src);

/// Printing operations

void dump1(pat_id id, PAT p);

void pat_dump();

/// The externally callable function that adds the pattern to the base by doing all the
/// pattern matching tasks.

int add(const PAT s);
int add(pat_id id, const PAT s);
};

}

#endif
