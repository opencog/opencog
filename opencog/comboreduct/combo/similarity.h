/*
 * opencog/comboreduct/combo/similarity.cc
 *
 * Copyright (C) 2014 Aidyia Limited
 * All Rights Reserved
 *
 * Written by Linas Vepstas
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

#ifndef COMBO_SIMILARITY_H_
#define COMBO_SIMILARITY_H_

#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <opencog/comboreduct/combo/vertex.h>

namespace opencog { namespace combo {

typedef std::map<std::string, unsigned> tree_branch_vector;
tree_branch_vector tree_flatten(const combo_tree&);
tree_branch_vector tree_flatten(const std::string& str);

size_t tree_similarity(const combo_tree&, const combo_tree&);
size_t tree_similarity(const tree_branch_vector&, const tree_branch_vector&);
size_t tree_similarity(const std::string&, const std::string&);

std::ostream& operator<<(std::ostream&, const tree_branch_vector&);
std::string toString(const tree_branch_vector& tbv)
{
	std::stringstream ss;
	ss << tbv;
	return ss.str();
}

} // ~namespace combo
} // ~namespace opencog

#endif // COMBO_SIMILARITY_H_
