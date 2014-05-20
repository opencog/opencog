/*
 * opencog/comboreduct/similarity/similarity.cc
 *
 * Copyright (C) 2014 Aidyia
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

#include <opencog/comboreduct/combo/vertex.h>

namespace opencog { namespace combo {

combo_tree to_binary_tree(const combo_tree&);

class tree_similarity
{
	public:
		tree_similarity(void);

};

} // ~namespace combo
} // ~namespace opencog

#endif // COMBO_SIMILARITY_H_
