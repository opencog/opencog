/*
 * opencog/spacetime/octomap/OctoMapNode.cc
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


#include <mutex>
#include <opencog/atoms/atom_types/NameServer.h>
#include <opencog/util/Logger.h>
#include <opencog/spacetime/octomap/atom_types.h>

#include "OctoMapNode.h"

using namespace opencog;

OctoMapNode::OctoMapNode(Type t, const std::string& name)
    : Node(t, name)
{
    //TODO make the params configurable.
    octomapPtr = std::make_shared<TimeOctomap<Handle>>(60, 0.001,
                 std::chrono::milliseconds(100));
}


OctoMapNode::OctoMapNode(const std::string& name, TimeOctomapPtr ocmap) : Node(OCTOMAP_NODE, name)
{
  octomapPtr = ocmap;
}

OctoMapNode::OctoMapNode(const Node& n)
    : Node(n)
{
    Type tocnode = n.get_type();
    if (not nameserver().isA(tocnode, OCTOMAP_NODE)) {
        const std::string& tname = nameserver().getTypeName(tocnode);
        throw InvalidParamException(TRACE_INFO,
                                    "Expecting an OctoMapNode, got %s", tname.c_str());
    }
}

OctoMapNode::~OctoMapNode()
{
}

/*Add factory.*/
DEFINE_NODE_FACTORY(OctoMapNode, OCTOMAP_NODE)
