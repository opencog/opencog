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

#include "AtomOcTreeNode.h"
#include "opencog/util/oc_assert.h"

namespace octomap
{


// node implementation  --------------------------------------
std::ostream&
AtomOcTreeNode::writeValue (std::ostream &s) const
{
    // 1 bit for each children; 0: empty, 1: allocated
    std::bitset<8> children;
    for (unsigned int i = 0; i < 8; i++) {
        if (this->childExists(i)) children[i] = 1;
        else                children[i] = 0;
    }
    char children_char = (char) children.to_ulong();

    // write node data
    s.write((const char*) &value, sizeof(value)); // occupancy
    s.write((const char*) &dat, sizeof(opencog::Handle)); // dat
    s.write((char*)&children_char, sizeof(char)); // child existence

    // write existing children
    for (unsigned int i = 0; i < 8; ++i)
        if (children[i] == 1) this->getChild(i)->writeValue(s);
    return s;
}

std::istream&
AtomOcTreeNode::readValue (std::istream &s)
{
    // read node data
    char children_char;
    s.read((char*) &value, sizeof(value)); // occupancy
    s.read((char*) &dat, sizeof(opencog::Handle)); // dat
    s.read((char*)&children_char, sizeof(char)); // child existence

    // read existing children
    std::bitset<8> children ((unsigned long long) children_char);
    for (unsigned int i = 0; i < 8; i++) {
        if (children[i] == 1) {
            createChild(i);
            getChild(i)->readValue(s);
        }
    }
    return s;
}
// pruning ============= mandeep: change dat here when pruned, depending on type of dat
bool
AtomOcTreeNode::pruneNode()
{
    // checks for equal occupancy only, dat ignored
    if (!this->collapsible()) return false;
    // set occupancy value
    setLogOdds(getChild(0)->getLogOdds());
    // set dat to average dat
    ////if (isColorSet()) dat = getAverageChildColor();//commented by mandeep
    dat = UndefinedHandle;
    // delete children
    for (unsigned int i = 0; i < 8; i++) {
        delete children[i];
    }
    delete[] children;
    children = nullptr;
    return true;
}

void
AtomOcTreeNode::expandNode()
{
    OC_ASSERT(!this->hasChildren());
    for (unsigned int k = 0; k < 8; k++) {
        this->createChild(k);
        this->children[k]->setValue(value);
        this->getChild(k)->setData(dat);
    }
}

} // end namespace

