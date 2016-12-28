/*
 *
 * Copyright (c) 2016, Mandeep Singh Bhatia, OpenCog Foundation
 * All rights reserved.
 * License: AGPL
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
