/*
 * opencog/spacetime/octomap/OctoMapNode.h
 *
 * Copyright (C) 2018 Hanson Robotics
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

#ifndef _OPENCOG_OCTOMAP_NODE_H
#define _OPENCOG_OCTOMAP_NODE_H
#include <memory>
#include <opencog/atoms/base/Node.h>

#include "TimeOctomap.h"

namespace opencog
{
/** \addtogroup grp_atomspace
 *  @{
 */

// OctomapNode is a wrapper class around the TimeOctoMap library
// and inherits from Node. The need for this wrapper arose while
// designing a new API for TimeOctoMap where cordinates are returned
// as FloatValues instead of atoms(which is more expensive both computationally
// and memory wise). Since these values must be persisited in the Time
// OctoMap via the API provided by the Value system, we needed a mechanism
// to directly access the TimeOctoMap library from Scheme without spending
// too much effort while creating a scheme binding for it. Creating an Atom
// wrapper around it was found to be the simplest solution which lets us reuse
// already existing Scheme interface for values and atoms and without additional
// effort.

using TimeOctomapPtr = std::shared_ptr<TimeOctomap<Handle>>;

class OctoMapNode : public Node
{
private:
   TimeOctomapPtr octomapPtr;
protected:

public:
    OctoMapNode(const std::string&);
    OctoMapNode(const std::string&, TimeOctomapPtr);
    OctoMapNode(const Node&);
    virtual ~OctoMapNode();

    inline TimeOctomapPtr get_map(void)
    {
        return octomapPtr;
    }
    static Handle factory(const Handle&);
};

typedef std::shared_ptr<OctoMapNode> OctoMapNodePtr;
static inline OctoMapNodePtr OctoMapNodeCast(const Handle& h)
{
    return std::dynamic_pointer_cast<OctoMapNode>(h);
}
static inline OctoMapNodePtr OctoMapNodeCast(AtomPtr a)
{
    return std::dynamic_pointer_cast<OctoMapNode>(a);
}

#define createOctoMapNode std::make_shared<OctoMapNode>

/** @}*/
}

#endif // _OPENCOG_OCTOMAP_NODE_H
