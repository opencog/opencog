/*
 * opencog/embodiment/Learning/behavior/BehaviorCategory.h
 *
 * Copyright (C) 2002-2009 Novamente LLC
 * All Rights Reserved
 * Author(s): Andre Senna
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


#ifndef BEHAVIORCATEGORY_H
#define BEHAVIORCATEGORY_H

#include "CompositeBehaviorDescription.h"
#include <string>
#include <vector>

namespace behavior
{

class BehaviorCategory
{

private:

    std::vector<CompositeBehaviorDescription> entries;
    //Note from Nil : apparently timelineSets and timelineIntervals
    //are not used so I commented them
    //std::vector<std::vector<Handle> > timelineSets;
    //std::vector<long> timelineIntervals;

    AtomSpace* atomspace;
public:

    // ***********************************************/
    // Constructors/destructors

    ~BehaviorCategory();
    BehaviorCategory(AtomSpace* a);

    // ***********************************************/
    // Building API

    void addCompositeBehaviorDescription(const CompositeBehaviorDescription &bd);

    // ***********************************************/
    // Manipulation API

    /**
     * @return the number of exemplars in BehaviorCategory
     */
    int getSize() const;

    const std::vector<CompositeBehaviorDescription> &getEntries() const;

    bool empty(); //check if empty
    void clear(); //clear the behavior category

    // ***********************************************/
    // Test/debug

    std::string toString();
    std::string toStringHandles();
    std::string toStringTimeline();

}; // class
}  // namespace

#endif
