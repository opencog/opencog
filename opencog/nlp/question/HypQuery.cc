/**
 * HypQuery.cc
 *
 * Impelement hypothetical query processing (yes/no questions)
 *
 * Copyright (c) 2008,2009 Linas Vepstas <linas@linas.org>
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

#include "HypQuery.h"

#include <stdio.h>

#include <opencog/atomspace/ForeachChaseLink.h>
#include <opencog/atomspace/Node.h>

using namespace opencog;

HypQuery::HypQuery(void)
{
	atom_space = NULL;
}

HypQuery::~HypQuery()
{
}

bool HypQuery::is_truth_query(Handle word_prop)
{
	Atom *atom = TLB::getAtom(word_prop);
	if (DEFINED_LINGUISTIC_CONCEPT_NODE != atom->getType()) return false;

	Node *n = static_cast<Node *>(atom);
	const std::string& name = n->getName();
	const char * str = name.c_str();
	if (0 == strcmp(str, "truth-query"))
		return true;
	return false;
}

/**
 * Return true if the indicated word-instance has the "truth-query" 
 * flag set.
 */
bool HypQuery::is_word_a_query(Handle word_inst)
{
	return foreach_binary_link(word_inst, INHERITANCE_LINK, &HypQuery::is_truth_query, this);
}

/* ============================= END OF FILE ======================== */
