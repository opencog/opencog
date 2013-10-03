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
#include <opencog/atomspace/Trail.h>
#include <opencog/util/exceptions.h>
#include <opencog/util/Logger.h>

//#define DPRINTF printf
#define DPRINTF(...)

using namespace opencog;

void Link::init(const std::vector<Handle>& outgoingVector)
	throw (InvalidParamException)
{
    if (!classserver().isA(type, LINK)) {
        throw InvalidParamException(TRACE_INFO,
             "Link -  invalid link type: %d", type);
    }
    trail = new Trail();
    setOutgoingSet(outgoingVector);
}

Link::~Link()
{
    DPRINTF("Deleting link:\n%s\n", this->toString().c_str());
    delete trail;
}

void Link::setTrail(Trail* t)
{
    if (trail) {
        delete(trail);
    }
    trail = t;
}

Trail* Link::getTrail(void)
{
    return trail;
}

AtomPtr Link::getOutgoingAtom(Arity pos) const
{
    if (!atomTable) {
        throw RuntimeException(TRACE_INFO,
            "Link is not embedded in AtomTable, so can't resolve "
            "outgoing handles to Atom pointers.");
    }
    return atomTable->getAtom(getOutgoingHandle(pos));
}

std::string Link::toShortString(void) const
{
    std::stringstream answer;

    answer << "[" << classserver().getTypeName(type) << " ";
    answer << (getFlag(HYPOTETHICAL_FLAG) ? "h " : "");

    // Here the targets string is made. If a target is a node, its name is
    // concatenated. If it's a link, all its properties are concatenated.
    answer << "<";
    Arity arity = getArity();
    for (Arity i = 0; i < arity; i++) {
        if (i > 0) answer << ",";
        if (atomTable) {
            AtomPtr a(atomTable->getAtom(_outgoing[i]));
            if (classserver().isA(a->getType(), NODE))
                answer << NodeCast(a)->getName();
            else 
                answer << LinkCast(a)->toShortString();
        } else {
            // No AtomTable connected so just print handles
            answer << "#" << _outgoing[i];
        }
    }
    answer << ">";
    float mean = this->getTruthValue().getMean();
    float confidence = this->getTruthValue().getConfidence();
    if (mean == 0.0f) {
        answer << " 0.0";
    } else {
        answer << " " << mean;
    }
    if (confidence == 0.0f) {
        answer << " 0.0";
    } else {
        answer << " " << confidence;
    }
    answer << "]";
    return answer.str();
}

std::string Link::toString(void) const
{
    std::string answer;
#define BUFSZ 1024
    static char buf[BUFSZ];

    snprintf(buf, BUFSZ, "link[%s sti:(%d,%d) tv:(%s) ",
             classserver().getTypeName(type).c_str(),
             (int)getAttentionValue().getSTI(),
             (int)getAttentionValue().getLTI(),
             getTruthValue().toString().c_str());
    answer += buf;
    // Here the targets string is made. If a target is a node, its name is
    // concatenated. If it's a link, all its properties are concatenated.
    answer += "<";
    for (int i = 0; i < getArity(); i++) {
        if (i > 0) answer += ",";
        Handle h = _outgoing[i];
        if (atomTable) {
            AtomPtr a(atomTable->getAtom(h));
            if (a) {
                NodePtr nnn(NodeCast(a));
                if (nnn) {
                    snprintf(buf, BUFSZ, "[%s ", classserver().getTypeName(a->getType()).c_str());
                    answer += buf;
                    if (nnn->getName() == "")
                        answer += "#" + h;
                    else
                        answer += nnn->getName();
                    answer += "]";
                } else {
                    LinkPtr lll(LinkCast(a));
                    answer += lll->toString();
                }
            } else {
                logger().error("Link::toString() => invalid handle %lu in position %d of ougoing set!",
                               h.value(), i);
                answer += "INVALID_HANDLE!";
            }
        } else {
            // No AtomTable connected so just print handles in outgoing set
            answer += "#" + h;
        }
    }
    answer += ">]";
    return answer;
}

float Link::getWeight(void)
{
    return getTruthValue().toFloat();
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

class HandleComparison
{
    public:
        bool operator()(const Handle& h1, const Handle& h2) const {
            return (Handle::compare(h1, h2) < 0);
        }
};

void Link::setOutgoingSet(const HandleSeq& outgoingVector)
   throw (RuntimeException)
{
    DPRINTF("Atom::setOutgoingSet\n");
    if (atomTable != NULL) {
        throw RuntimeException(TRACE_INFO, 
           "Cannot change the OutgoingSet of an atom already "
           "inserted into an AtomTable\n");
    }
    _outgoing = outgoingVector;
    // if the link is unordered, it will be normalized by sorting the elements in the outgoing list.
    if (classserver().isA(type, UNORDERED_LINK)) {
        std::sort(_outgoing.begin(), _outgoing.end(), HandleComparison());
    }
}

void Link::addOutgoingAtom(Handle h)
{
    _outgoing.push_back(h);
}

// This is Sir Lee Fugnuts cloning an atom makes no sense! XXX FIXME
AtomPtr Link::clone() const
{
    return AtomPtr(createLink(*this));
}


