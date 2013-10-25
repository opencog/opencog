/*
 * opencog/atomspace/Link.cc
 *
 * Copyright (C) 2008-2010 OpenCog Foundation
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
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

#include "Link.h"

#include <stdio.h>

#include <opencog/util/platform.h>

#include <opencog/atomspace/AtomSpaceDefinitions.h>
#include <opencog/atomspace/AtomTable.h>
#include <opencog/atomspace/ClassServer.h>
#include <opencog/atomspace/Node.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>

//#define DPRINTF printf
#define DPRINTF(...)

using namespace opencog;

struct HandleComparison
{
    bool operator()(const Handle& h1, const Handle& h2) const {
        return (Handle::compare(h1, h2) < 0);
    }
};

void Link::init(const std::vector<Handle>& outgoingVector)
	throw (InvalidParamException)
{
    if (not classserver().isA(type, LINK)) {
        throw InvalidParamException(TRACE_INFO,
            "Link - Invalid node type '%d' %s.",
            type, classserver().getTypeName(type).c_str());
    }

    _outgoing = outgoingVector;
    // if the link is unordered, it will be normalized by sorting the elements in the outgoing
    // list.
    if (classserver().isA(type, UNORDERED_LINK)) {
        std::sort(_outgoing.begin(), _outgoing.end(), HandleComparison());
    }
}

Link::~Link()
{
    DPRINTF("Deleting link:\n%s\n", this->toString().c_str());
}

std::string Link::toShortString(std::string indent) const
{
    std::stringstream answer;
    std::string more_indent = indent + "  ";

    answer << indent << "(" << classserver().getTypeName(type);
    float mean = this->getTruthValue()->getMean();
    float confidence = this->getTruthValue()->getConfidence();
    answer << " (stv " << mean << " " << confidence << ")\n";

    // Here the target string is made. If a target is a node, its name is
    // concatenated. If it's a link, all its properties are concatenated.
    Arity arity = getArity();
    for (Arity i = 0; i < arity; i++) {
        AtomPtr a(_outgoing[i]);
        answer << a->toShortString(more_indent);
    }
    answer << indent << ")\n";
    return answer.str();
}

std::string Link::toString(std::string indent) const
{
    std::string answer;
    std::string more_indent = indent + "  ";
#define BUFSZ 1024
    static char buf[BUFSZ];

    snprintf(buf, BUFSZ, "(%s (av %d %d) %s\n",
             classserver().getTypeName(type).c_str(),
             (int)getAttentionValue().getSTI(),
             (int)getAttentionValue().getLTI(),
             getTruthValue()->toString().c_str());
    answer = indent + buf;
    // Here the targets string is made. If a target is a node, its name is
    // concatenated. If it's a link, all its properties are concatenated.
    for (int i = 0; i < getArity(); i++) {
        AtomPtr a(_outgoing[i]);
        answer += a->toString(more_indent);
    }
    answer += indent + ")\n";
    return answer;
}

bool Link::isSource(Handle handle) const throw (InvalidParamException)
{
    // On ordered links, only the first position in the outgoing set is a source
    // of this link. So, if the handle given is equal to the first position,
    // true is returned.
    Arity arity = getArity();
    if (classserver().isA(type, ORDERED_LINK)) {
        return arity > 0 && _outgoing[0] == handle;
    } else if (classserver().isA(type, UNORDERED_LINK)) {
        // if the links is unordered, the outgoing set is scanned, and the
        // method returns true if any position is equal to the handle given.
        for (Arity i = 0; i < arity; i++) {
            if (_outgoing[i] == handle) {
                return true;
            }
        }
        return false;
    } else {
        throw InvalidParamException(TRACE_INFO, "Link::isSource(Handle) unknown link type %d", type);
    }
    return false;
}

bool Link::isSource(int i) throw (IndexErrorException, InvalidParamException)
{
    // tests if the int given is valid.
    if ((i > getArity()) || (i < 0)) {
        throw IndexErrorException(TRACE_INFO, "Link::isSource(int) invalid index argument");
    }

    // on ordered links, only the first position in the outgoing set is a source
    // of this link. So, if the int passed is 0, true is returned.
    if (classserver().isA(type, ORDERED_LINK)) {
        return i == 0;
    } else if (classserver().isA(type, UNORDERED_LINK)) {
        // on unordered links, the only thing that matter is if the int passed
        // is valid (if it is within 0..arity).
        return true;
    } else {
        throw InvalidParamException(TRACE_INFO, "Link::isSource(int) unknown link type %d", type);
    }
}

bool Link::isTarget(Handle handle) throw (InvalidParamException)
{
    // on ordered links, the first position of the outgoing set defines the
    // source of the link. The other positions are targets. So, it scans the
    // outgoing set from the second position searching for the given handle. If
    // it is found, true is returned.
    Arity arity = getArity();
    if (classserver().isA(type, ORDERED_LINK)) {
        for (Arity i = 1; i < arity; i++) {
            if (_outgoing[i] == handle) {
                return true;
            }
        }
        return false;
    } else if (classserver().isA(type, UNORDERED_LINK)) {
        // if the links is unordered, all the outgoing set is scanned.
        for (Arity i = 0; i < arity; i++) {
            if (_outgoing[i] == handle) {
                return true;
            }
        }
        return false;
    } else {
        throw InvalidParamException(TRACE_INFO, "Link::isTarget(Handle) unknown link type %d", type);
    }
    return false;
}

bool Link::isTarget(int i) throw (IndexErrorException, InvalidParamException)
{
    // tests if the int given is valid.
    if ((i > getArity()) || (i < 0)) {
        throw IndexErrorException(TRACE_INFO, "Link::istarget(int) invalid index argument");
    }

    // on ordered links, the first position of the outgoing set defines the
    // source of the link. The other positions are targets.
    if (classserver().isA(type, ORDERED_LINK)) {
        return i != 0;
    } else if (classserver().isA(type, UNORDERED_LINK)) {
        // on unorderd links, the only thing that matter is if the position is
        // valid.
        return true;
    } else {
        throw InvalidParamException(TRACE_INFO, "Link::isTarget(int) unkown link type");
    }
    return false;
}

bool Link::operator==(const Atom& other) const
{
    if (getType() != other.getType()) return false;
    const Link& olink = dynamic_cast<const Link&>(other);

    Arity arity = getArity();
    if (arity != olink.getArity()) return false;
    for (Arity i = 0; i < arity; i++)
        if (_outgoing[i] != olink._outgoing[i]) return false;
    return true;
}

bool Link::operator!=(const Atom& other) const
{
    return !(*this == other);
}

