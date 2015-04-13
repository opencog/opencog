/*
 * opencog/embodiment/Learning/behavior/PredicateHandleSet.cc
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


#include <opencog/atomspace/Node.h>
#include "PredicateHandleSet.h"

using namespace behavior;

PredicateHandleSet::~PredicateHandleSet()
{
}

PredicateHandleSet::PredicateHandleSet()
{
}

void PredicateHandleSet::insert(const Handle &handle)
{
    handles.insert(handle);
}

void PredicateHandleSet::clear()
{
    handles.clear();
}

const std::set<Handle> &PredicateHandleSet::getSet() const
{
    return handles;
}

bool PredicateHandleSet::equals(const PredicateHandleSet &other) const
{

    if (other.getSize() != this->getSize()) {
        return false;
    }

    std::set<Handle>::iterator it1, it2;
    for (it1 = handles.begin(), it2 = other.handles.begin(); it1 != handles.end(); ++it1, ++it2) {
        if (*it1 != *it2) {
            return false;
        }
    }

    return true;
}

std::string PredicateHandleSet::toString(AtomSpace &atomspace) const
{

    std::string answer = "{";
    std::set<Handle>::iterator it = handles.begin();
    while (it != handles.end()) {
        //if Atom then print the name
        //but if link then print the string representation
        //printing the atom name instead of its string representation
        //is kept for compatibility reason
        std::string str;
        if (atomspace.isNode(*it)) {
            const std::string an = atomspace.getName(*it);
            str = an;
        } else {
            str = atomspace.atomAsString((*it));
        }
        answer.append(str);
        ++it;
        if (it != handles.end()) {
            answer.append(",");
        }
    }
    answer.append("}");

    return answer;
}

bool PredicateHandleSet::operator==(const PredicateHandleSet& other) const
{

    //printf("operator %s == %s\n", toString().c_str(), other.toString().c_str());

    bool answer = equals(other);

    /*
    if (answer) {
        printf("true\n");
    } else {
        printf("false\n");
    }
    */

    return answer;
}

PredicateHandleSet &PredicateHandleSet::operator=(const PredicateHandleSet & other)
{

    //printf("operator %s = %s\n", toString().c_str(), other.toString().c_str());

    handles.clear();
    for (std::set<Handle>::iterator it = other.handles.begin(); it != other.handles.end(); ++it) {
        handles.insert(*it);
    }
    //printf("exiting operator %s = %s\n", toString().c_str(), other.toString().c_str());
    return *this;
}

bool PredicateHandleSet::operator<(const PredicateHandleSet& other) const
{

    //printf("operator %s < %s\n", toString().c_str(), other.toString().c_str());

    if (getSize() == 0) {
        if (other.getSize() == 0) {
            return false;
        } else {
            return true;
        }
    } else {
        if (other.getSize() == 0) {
            return false;
        }
    }

    std::set<Handle>::iterator it1, it2;
    for (it1 = handles.begin(), it2 = other.handles.begin(); ((it1 != handles.end()) && (it2 != other.handles.end())); ) {
        if ((*it1) < (*it2)) {
            //printf("true 1\n");
            return true;
        } else {
            if ((*it1) > (*it2)) {
                //printf("false 1\n");
                return false;
            } else {
                ++it1;
                ++it2;
                if (it1 == handles.end()) {
                    if (it2 == other.handles.end()) {
                        //printf("false 2\n");
                        return false;
                    } else {
                        //printf("true 2\n");
                        return true;
                    }
                } else {
                    if (it2 == other.handles.end()) {
                        //printf("false 3\n");
                        return false;
                    }
                }
            }
        }
    }

    //printf("false 4\n");
    return false;
}

int PredicateHandleSet::getSize() const
{
    return handles.size();
}

bool PredicateHandleSet::empty() const
{
    return handles.empty();
}

