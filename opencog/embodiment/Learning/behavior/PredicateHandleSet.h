/*
 * opencog/embodiment/Learning/behavior/PredicateHandleSet.h
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


#ifndef PREDICATEHANDLESET_H
#define PREDICATEHANDLESET_H

#include <set>
#include <string>
#include <opencog/atomspace/types.h>
#include <opencog/atomspace/AtomSpace.h>

using namespace opencog;

namespace behavior
{

// TODO: this class should extend std::set
class PredicateHandleSet
{

private:

    std::set<Handle> handles;

public:

    // ***********************************************/
    // Constructors/destructors

    virtual ~PredicateHandleSet();
    PredicateHandleSet();

    // ***********************************************/
    // API

    void insert(const Handle &handle);
    void clear();
    const std::set<Handle> &getSet() const;
    int getSize() const;
    bool empty() const;
    bool equals(const PredicateHandleSet &other) const;
    std::string toString(AtomSpace& atomspace) const;
    virtual PredicateHandleSet &operator=(const PredicateHandleSet& other);
    virtual bool operator==(const PredicateHandleSet& other) const;
    virtual bool operator<(const PredicateHandleSet& other) const;
}; // class
}  // namespace

#endif
