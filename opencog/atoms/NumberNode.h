/*
 * opencog/execution/NumberNode.h
 *
 * Copyright (C) 2015 Linas Vepstas
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

#ifndef _OPENCOG_NUMBER_NODE_H
#define _OPENCOG_NUMBER_NODE_H

#include <opencog/atomspace/AtomSpace.h>
#include <opencog/atomspace/Node.h>

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 *
 * Experimental NumberNode class. This is a rough sketch for how things
 * like this might be done. It is not necessarily a god idea, and might
 * be replaced by something completely different, someday ...
 */

class NumberNode : public Node
{
protected:
	double value;

public:
	NumberNode(const std::string& s,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV())
		// Convert to number and back to string to avoid miscompares.
		: Node(NUMBER_NODE, std::to_string(std::stod(s)), tv, av),
		  value(std::stod(s))
	{}

	NumberNode(double vvv,
	           TruthValuePtr tv = TruthValue::DEFAULT_TV(),
	           AttentionValuePtr av = AttentionValue::DEFAULT_AV())
		: Node(NUMBER_NODE, std::to_string(vvv), tv, av),
		  value(vvv)
	{}

	NumberNode(Node &n)
		: Node(NUMBER_NODE, std::to_string(std::stod(n.getName())),
		       n.getTruthValue(), n.getAttentionValue()),
		  value(std::stod(n.getName()))
	{
		OC_ASSERT(NUMBER_NODE == n.getType(), "Bad NumberNode consructor!");
	}

	double getValue(void) { return value; }
};

typedef std::shared_ptr<NumberNode> NumberNodePtr;
static inline NumberNodePtr NumberNodeCast(const Handle& h)
	{ AtomPtr a(h); return std::dynamic_pointer_cast<NumberNode>(a); }
static inline NumberNodePtr NumberNodeCast(AtomPtr a)
	{ return std::dynamic_pointer_cast<NumberNode>(a); }

// XXX temporary hack ...
#define createNumberNode std::make_shared<NumberNode>

/** @}*/
}

#endif // _OPENCOG_NUMBER_NODE_H
