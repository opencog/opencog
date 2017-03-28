/*
 * Copyright (c) 2016, Mandeep Singh Bhatia, OpenCog Foundation
 *
 * All rights reserved.
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

#ifndef TEMPLATE_OCTREE_NODE_H
#define TEMPLATE_OCTREE_NODE_H

#include <iostream>
#include <octomap/OcTreeNode.h>
#include <opencog/atomspace/AtomSpace.h>
namespace octomap
{
//typedef opencog::Handle opencog::Handle;
const opencog::Handle UndefinedHandle = opencog::Handle::UNDEFINED;
// node definition
class AtomOcTreeNode : public OcTreeNode
{
public:
    AtomOcTreeNode() : OcTreeNode()
    {} //dat gets default value from prunning

    AtomOcTreeNode(const AtomOcTreeNode& rhs) : OcTreeNode(rhs), dat(rhs.dat)
    {}

    bool operator==(const AtomOcTreeNode& rhs) const
    {
        return (rhs.value == value && rhs.dat == dat);
    }

    // children
    inline AtomOcTreeNode* getChild(unsigned int i)
    {
        return static_cast<AtomOcTreeNode*> (OcTreeNode::getChild(i));
    }
    inline const AtomOcTreeNode* getChild(unsigned int i) const
    {
        return static_cast<const AtomOcTreeNode*> (OcTreeNode::getChild(i));
    }

    bool createChild(unsigned int i)
    {
        if (children == nullptr) allocChildren();
        children[i] = new AtomOcTreeNode();
        return true;
    }

    bool pruneNode();
    void expandNode();

    inline opencog::Handle getData() const
    {
        return dat;
    }
    inline void  setData(opencog::Handle c)
    {
        this->dat = c;
    }

    opencog::Handle& getData()
    {
        return dat;
    }

    // file I/O
    std::istream& readValue (std::istream &s);
    std::ostream& writeValue(std::ostream &s) const;

protected:
    opencog::Handle dat;
};

} // end namespace

#endif
