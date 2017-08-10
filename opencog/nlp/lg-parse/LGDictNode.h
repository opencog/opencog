/*
 * opencog/nlp/lg-dict/LGDictNode.h
 *
 * Copyright (C) 2017 Linas Vepstas
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the
 * exceptions at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public
 * License along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _OPENCOG_LG_DICT_NODE_H
#define _OPENCOG_LG_DICT_NODE_H

#include <string>
#include <link-grammar/dict-api.h>
#include <opencog/atoms/base/Node.h>

namespace opencog
{

/** \addtogroup grp_atomspace
 *  @{
 */

/// The LgDictNode represents a Link Grammar Dictionary. It is an
/// external data source that certain subsystems need to access to
/// obtain grammatical data.  The Node holds a pointer to the Link
/// Grammar Dictionary itself, so that it can be directly accessed.

class LgDictNode : public Node
{
protected:
	Dictionary _dict;

public:
	LgDictNode(const std::string&);
	LgDictNode(const Node&);
	virtual ~LgDictNode();

	Dictionary get_dictionary(void);

	static Handle factory(const Handle&);
};

typedef std::shared_ptr<LgDictNode> LgDictNodePtr;
static inline LgDictNodePtr LgDictNodeCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<LgDictNode>(a); }
static inline LgDictNodePtr LgDictNodeCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<LgDictNode>(a); }

// XXX temporary hack ...
#define createLgDictNode std::make_shared<LgDictNode>

/** @}*/
}

#endif // _OPENCOG_LG_DICT_NODE_H
