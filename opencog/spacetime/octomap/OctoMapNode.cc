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

#include <opencog/atoms/value/NameServer.h>
#include <opencog/util/Logger.h>
#include <opencog/spacetime/octomap/atom_types.h>

#include "OctoMapNode.h"

using namespace opencog;

OctoMapNode::OctoMapNode(const std::string& name)
    : Node(OCTOMAP_NODE, name)
{
    //TODO make the params configurable.
    octomapPtr = std::make_shared<TimeOctomap<Handle>>(60, 0.001,
                 std::chrono::milliseconds(100));
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

Handle OctoMapNode::factory(const Handle& base)
{
    if (OctoMapNodeCast(base)) return base;
    Handle h(createOctoMapNode(base->get_name()));
    return h;
}

/* This runs when the shared lib is loaded. */
static __attribute__ ((constructor)) void init(void)
{
    classserver().addFactory(OCTOMAP_NODE, &OctoMapNode::factory);
}

/* This allows guile to load this shared library */
extern "C" {
    void opencog_spacetime_octomap_init(void) {}
};
