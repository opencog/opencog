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

